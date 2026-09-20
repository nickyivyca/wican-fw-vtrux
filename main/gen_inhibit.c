#include "gen_inhibit.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"

#include "can.h"

static const char *TAG = "gen_inhibit";

/*
 * Priority. wican-fw's can_rx_task and can_tx_task run at 5; the WiFi driver
 * task is 23. We sit above the wican tasks so their queue work cannot delay a
 * response, and below WiFi so we do not starve the stack we depend on for
 * recovery.
 */
#define GEN_INHIBIT_TASK_PRIO   18
#define GEN_INHIBIT_STACK       (1024 * 4)

/*
 * Auto-arm on boot (build-time).
 *
 *   GEN_INHIBIT_OFF     boot idle; arm manually via POST /gen_inhibit_set.
 *                       THE DEFAULT -- used for bench and truck bring-up, so the
 *                       device transmits nothing until a human arms it.
 *   GEN_INHIBIT_INHIBIT arm INHIBIT as soon as the worker starts: relay power ->
 *                       boot -> inhibiting, no human in the loop. The deployed
 *                       configuration. The Via harness powers the dongle at
 *                       truck-wake, well before any crank, so this arms in time.
 *   GEN_INHIBIT_OBSERVE / _RESPOND also valid (measurement builds).
 *
 * Safe at any setting: reactive trail transmits only in response to a received
 * 0x051, so an armed-but-idle bus produces no traffic. Override at build time
 * with -DGEN_INHIBIT_AUTOARM_MODE=... via target_compile_definitions (a plain
 * idf.py -D sets a CMake cache var, not a macro).
 */
#ifndef GEN_INHIBIT_AUTOARM_MODE
#define GEN_INHIBIT_AUTOARM_MODE        GEN_INHIBIT_OFF
#endif
#ifndef GEN_INHIBIT_AUTOARM_OFFSET_US
#define GEN_INHIBIT_AUTOARM_OFFSET_US   500
#endif

/*
 * Conditions that DISABLE the inhibit, latched until reboot. Both mean the
 * engine/generator is legitimately wanted, so we stop transmitting and stay
 * stopped (power-cycle -- the relay drops at truck sleep -- to re-enable).
 *
 *   M mode  -- driver manually commanding the engine (0x639 shift_lever_pos==4,
 *              "Manual_generator_mode", confirmed in the powertrain DBC). Both
 *              0x639 and 0x051 are on the PT bus, so the worker sees it inline.
 *   Low SoC -- pack needs the generator to charge. 0x411 BMS_SoC_HiRes, a 14-bit
 *              big-endian value at bit 7, scale 0.01%: raw=(B0<<6)|(B1>>2),
 *              percent = raw/100.
 *
 * Sentinel resilience: 0x411 reads 0 for the first samples at startup before it
 * is valid. A raw 0 is treated as "not yet valid", never as 0%, and the
 * sub-threshold condition must persist GEN_INHIBIT_SOC_DEBOUNCE valid samples
 * before it latches -- so a transient 0->real-value at boot cannot false-trip.
 */
#define GEN_INHIBIT_MMODE_ID        0x639
#define GEN_INHIBIT_MMODE_VALUE     4
#define GEN_INHIBIT_SOC_ID          0x411
#ifndef GEN_INHIBIT_SOC_MIN_PCT
/*
 * 21 %, deliberately a full percentage point above the truck's own 20.0 %
 * trigger, so we get out of the way BEFORE the VCM asks for the engine rather
 * than at the same instant. The cost is giving up inhibiting in the 20-21 %
 * band; the benefit is that there is no window in which we are blocking a
 * start the pack genuinely needs. Spec section 6.2: isolating entries into
 * charge-sustain across the whole corpus leaves two events, firing at 20.02 %
 * and 20.23 %, so the truck defends a 20.0 % floor.
 */
#define GEN_INHIBIT_SOC_MIN_PCT     21
#endif
#define GEN_INHIBIT_SOC_MIN_RAW     (GEN_INHIBIT_SOC_MIN_PCT * 100)
#define GEN_INHIBIT_SOC_DEBOUNCE    5

/*
 * 0x051 B0, the VCM's generator-state command byte. Three values across the
 * corpus (spec 4.1):
 *
 *   0x08  engine off / idle      OPERATIONAL on 0x053, torque zero
 *   0x0B  start requested        OPERATIONAL on 0x053, torque nonzero
 *   0x10  shutdown commanded     SHTDWN on 0x053, 99.0 %
 *
 * We transmit only 0x08 (spec 4.1) and we suppress transmission entirely while
 * the VCM is sending 0x10 (spec 6.3).
 */
#define GEN_INHIBIT_B0_ENGINE_OFF   0x08
#define GEN_INHIBIT_B0_SHUTDOWN     0x10

/*
 * Spec 7: the laptop tool's arm interlocks, ported. Every one of these fired,
 * or would have, during real truck testing.
 *
 * Signals, all against the DBC rather than the reference tool -- see the note
 * on GEN_INHIBIT_RPM_ZERO below, where the two disagree:
 *
 *   0x051 B1-B2  gen_torque_cmd  LE, offset -32768   (VCM commanding torque?)
 *   0x051 B3-B4  gen_rpm_ref     LE, offset -32768, -1 = engine off
 *   0x471        GENE inverter feedback -- liveness only, value unused
 *   0x054 B0-B1  GENE_RotSpd     LE, offset -32767   (did the engine turn?)
 *   0x617 B7     VCM fault flag, 0xCA = FAULT ACTIVE
 */
#define GEN_INHIBIT_FB_ID           0x471
#define GEN_INHIBIT_RPM_ID          0x054
#define GEN_INHIBIT_FAULT_ID        0x617
#define GEN_INHIBIT_FAULT_ACTIVE    0xCA

/*
 * GENE_RotSpd is the odd one out and the DBC says so explicitly:
 *   SG_ GENE_RotSpd : 0|16@1+ (1,-32767)
 *   CM_ "Zero at raw=32767 (0x7FFF). Note: GENE uses 32767, not 32768 like
 *        trac_rpm."
 * The truck-validated Python reference decodes it with its generic le16c(),
 * which subtracts 32768, so it reads this one signal 1 rpm low and reports
 * engine-off as -1 rather than 0. Immaterial against a 300 rpm threshold, but
 * the DBC is the source of truth for encoding, so this follows the DBC.
 */
#define GEN_INHIBIT_RPM_ZERO        32767
#define GEN_INHIBIT_CMD_ZERO        32768

/*
 * 300 rpm sustained for 0.3 s, from the laptop tool's --start-abort-rpm. The
 * debounce is not optional: a momentary crank blip is exactly what a WORKING
 * inhibit produces, and aborting on it would end the run the inhibit just won.
 */
#define GEN_INHIBIT_START_ABORT_RPM 300
#define GEN_INHIBIT_RPM_DEBOUNCE_US 300000

/* Freshness, not last-known values: a silent bus otherwise answers every
 * question you ask it. 0.5 s matches the reference tool's gene_alive(). */
#define GEN_INHIBIT_FRESH_US        500000


/*
 * Error frames are rate-based, never first-strike. Aborting on the first one
 * ended three consecutive armed runs. The running-generator regime produces
 * 0.07-0.21/s and bursts to 4 in any 10 s; a background measured engine-off
 * reads ~0/s and is useless for calibration. 10 in 10 s keeps 2.5x headroom
 * over the worst observed burst, while a genuine same-ID collision at 100 Hz
 * would blow past it inside a second.
 */
#define GEN_INHIBIT_ERR_WINDOW_US   10000000LL
#define GEN_INHIBIT_ERR_MIN_TRIP    10

/*
 * Self-telemetry frames, so an Android/laptop log is self-documenting: which
 * build ran, what mode, whether/why disabled, how much it transmitted. Layout
 * and versions: notes/artifacts/gen-inhibit/wican_diag_schema.md and
 * projects/vtrux/vtrux-wican-diag.dbc. Three fixed IDs (no paging), round-robin
 * at ~1 Hz each while armed. 0x7F1-0x7F3 are in the 0x7Fx range nothing on the
 * truck consumes (same rationale as the 0x7F0 probe).
 */
#define GEN_INHIBIT_DIAG_ID_STATUS   0x7F1
#define GEN_INHIBIT_DIAG_ID_COUNTERS 0x7F2
#define GEN_INHIBIT_DIAG_ID_BUILD    0x7F3
/* 2: added diag_flags bit4, the non-latching 0x10 shutdown suppression. */
#define GEN_INHIBIT_DIAG_SCHEMA_VER  2
#ifndef DIAG_FW_VERSION
#define DIAG_FW_VERSION              1
#endif
#define GEN_INHIBIT_DIAG_PERIOD_MS   300
#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

/*
 * Latency histogram, microseconds. Boundaries are chosen around the decision
 * we actually face: under ~200 us a plain high-priority task is enough and the
 * IRAM-resident design is unnecessary complexity; past ~1 ms we are eating the
 * guard band and need the interrupt path.
 */
#define NBUCKETS 10
static const uint32_t s_bucket_us[NBUCKETS] = {
    50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, 0xFFFFFFFF
};

typedef struct
{
    uint32_t count;
    uint32_t min_us;
    uint32_t max_us;
    uint64_t sum_us;
    uint32_t buckets[NBUCKETS];
} hist_t;

static hist_t s_rx_gap;     /* 0x051 inter-arrival, as the ESP32 sees it */
static hist_t s_response;   /* RX of 0x051 -> probe frame queued */

static volatile gen_inhibit_mode_t s_mode = GEN_INHIBIT_OFF;
static volatile uint32_t s_offset_us = 500;

static uint32_t s_tx_ok;
static uint32_t s_tx_fail;
static uint32_t s_other_frames;
static uint8_t  s_last_ctr;
static uint32_t s_ctr_steps_ok;
static uint32_t s_ctr_steps_bad;
static bool     s_have_last_ctr;
static bool     s_we_enabled_bus;   /* we called can_enable(), so we may disable */
static volatile bool s_release_bus; /* disarm asked us to hand the bus back */
static uint32_t s_rx_errors;        /* twai_receive failures that were not timeouts */

static TaskHandle_t  s_worker;      /* the worker task, for self-call detection */
static volatile bool s_quiesce;     /* an external caller needs the driver freed */
static volatile bool s_parked;      /* worker is provably outside twai_receive() */

static volatile bool s_disabled;        /* latched: inhibit off until reboot */
static const char   *s_disable_reason = "";
static uint8_t       s_disable_code;    /* 0 none, 1 m_mode, 2 low_soc (for diag) */
static uint32_t      s_soc_raw;         /* last valid SoC, raw (percent*100) */
static uint32_t      s_soc_low_count;   /* consecutive valid sub-threshold samples */
static uint8_t       s_last_shift_pos = 0xFF;  /* last 0x639 shift_lever_pos */
static uint32_t      s_git_hash;        /* FNV-1a of GIT_SHA, computed at init */

/*
 * Spec 6.3: the VCM is commanding generator shutdown (0x051 B0 = 0x10), so we
 * stay quiet. NOT latched, and deliberately the odd one out next to s_disabled
 * -- 0x10 is a parked state the truck passes through and drives out of again
 * (measured: 40,697 frames of it, 100 % with the shifter in Park, zero while
 * moving, up to four episodes in one capture). Latching here would stand the
 * inhibitor down for the whole subsequent drive, which is the drive it exists
 * for. It also removes any need for a startup guard, since 0x10 is the normal
 * state at key-on and a non-latching suppression simply waits it out.
 */
static volatile bool s_shutdown_suppressed;

/*
 * Spec 13 item 1: we put the controller into listen-only for OBSERVE. Remember
 * that we did, and what the configured value was, so leaving OBSERVE restores
 * the user's setting rather than asserting one of our own.
 */
static bool     s_forced_silent;
static uint8_t  s_silent_saved;

/*
 * Spec 7 interlock state.
 *
 * `s_inhibit_live` is the distinction that makes this work on a device with no
 * operator: mode INHIBIT is what was *requested*, s_inhibit_live is whether the
 * interlocks currently permit transmitting. The laptop tool could refuse to arm
 * because a human was there to read the refusal; the WiCAN may auto-arm at
 * truck-wake with nobody in the loop, and it has no frames at all in the
 * instant it arms. So the gate is evaluated continuously and the inhibit goes
 * live only once every condition holds. That is also why auto-arm is safe: the
 * trail is reactive, and until the gate passes there is nothing to react with.
 */
static volatile bool s_inhibit_live;
static const char   *s_arm_block = "not armed";   /* why not live, if not */
static const char   *s_abort_reason = "";         /* why a live run ended */

static int64_t s_seen_cmd;      /* last 0x051, us; 0 = never */
static int64_t s_seen_fb;       /* last 0x471 */
static bool    s_fb_ever;       /* 0x471 heard at least once this run */
static int64_t s_seen_rpm;      /* last 0x054 */
static int32_t s_vcm_torque;    /* 0x051 B1-B2, centred */
static int32_t s_vcm_rpm_ref;   /* 0x051 B3-B4, centred; -1 = engine off */
static int32_t s_gene_rpm;      /* 0x054 B0-B1, centred per the DBC */
static uint8_t s_vcm_fault = 0xC8;
static int64_t s_rpm_over_since;/* debounce anchor for the engine-turning trip */
static uint32_t s_err_base;     /* bus_error_count at the window start */
static int64_t s_err_window;    /* start of the current error-rate window */

/* Defined below, next to the rest of the interlocks; send_inhibit() needs it
 * for the failed-transmit trip and comes first in the file. */
static void inhibit_abort(const char *why);

/*
 * A run of hard receive errors means the driver is gone underneath us -- for
 * example another subsystem called can_disable(). Disarm rather than keep
 * retrying, so the device parks itself in a safe, answerable state instead of
 * needing a power cycle.
 */
#define GEN_INHIBIT_MAX_RX_ERRORS   20

static void hist_reset(hist_t *h)
{
    memset(h, 0, sizeof(*h));
    h->min_us = 0xFFFFFFFF;
}

static void hist_add(hist_t *h, uint32_t us)
{
    h->count++;
    h->sum_us += us;
    if (us < h->min_us) h->min_us = us;
    if (us > h->max_us) h->max_us = us;
    for (int i = 0; i < NBUCKETS; i++)
    {
        if (us < s_bucket_us[i])
        {
            h->buckets[i]++;
            return;
        }
    }
}

static int hist_json(const hist_t *h, const char *name, char *buf, int buflen)
{
    int n = snprintf(buf, buflen,
                     "\"%s\":{\"count\":%lu,\"min_us\":%lu,\"max_us\":%lu,\"mean_us\":%lu,\"buckets\":[",
                     name, (unsigned long)h->count,
                     (unsigned long)(h->count ? h->min_us : 0),
                     (unsigned long)h->max_us,
                     (unsigned long)(h->count ? (uint32_t)(h->sum_us / h->count) : 0));
    for (int i = 0; i < NBUCKETS && n < buflen; i++)
    {
        n += snprintf(buf + n, buflen - n, "%s%lu",
                      (i ? "," : ""), (unsigned long)h->buckets[i]);
    }
    if (n < buflen)
    {
        n += snprintf(buf + n, buflen - n, "]}");
    }
    return n;
}

void gen_inhibit_reset_stats(void)
{
    hist_reset(&s_rx_gap);
    hist_reset(&s_response);
    s_tx_ok = s_tx_fail = s_other_frames = 0;
    s_ctr_steps_ok = s_ctr_steps_bad = 0;
    s_have_last_ctr = false;
    s_rx_errors = 0;
    /*
     * Live bus-derived state, not a statistic, but it must not survive an arm
     * cycle: disarming mid-0x10 and re-arming would otherwise start suppressed
     * on a stale reading until the next 0x051. It is recomputed from the first
     * frame either way; this just makes the starting point honest.
     */
    s_shutdown_suppressed = false;

    /*
     * Spec 7: a new arm starts from a clean gate. Signal freshness is
     * deliberately NOT cleared -- it is a property of the bus, not of this
     * run, and dropping it would make every arm wait a fresh round before the
     * interlocks could pass.
     */
    s_inhibit_live = false;
    s_arm_block = "gate not yet evaluated";
    s_abort_reason = "";
    s_rpm_over_since = 0;
    s_err_window = 0;
    s_fb_ever = false;
}

bool gen_inhibit_owns_bus(void)
{
    return s_mode != GEN_INHIBIT_OFF;
}

esp_err_t gen_inhibit_set_mode(gen_inhibit_mode_t mode, uint32_t offset_us)
{
    if (mode > GEN_INHIBIT_INHIBIT)
    {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * A probe scheduled beyond the VCM's shortest observed inter-frame gap
     * (4.69 ms, per notes/gen-inhibit-truck-tests.md) would land after the
     * next genuine frame and make the measurement meaningless.
     */
    if (offset_us > 4000)
    {
        return ESP_ERR_INVALID_ARG;
    }
    /*
     * Bring the bus up ourselves. can_init() only records the bitrate -- it is
     * can_enable() that installs the TWAI driver, calls twai_start() and drops
     * the transceiver's STBY line. In the stock firmware that happens when a
     * client opens the SLCAN socket, so on an idle WiCAN the driver is *not*
     * installed and twai_receive() returns ESP_ERR_INVALID_STATE immediately.
     *
     * Arming without this wedged the device: the worker spun on that error at
     * priority 18, starving httpd (5) and can_tx_task (5). The AP stayed up
     * and TCP still accepted, because lwIP also runs at 18 and round-robins
     * with us -- so the device looked alive and answered nothing. Recovery
     * needed a power cycle, on a board with no serial console.
     */
    if (mode != GEN_INHIBIT_OFF)
    {
        /* Cancel any standing quiesce so the worker resumes into the driver. */
        s_quiesce = false;
        s_parked = false;

        /*
         * Spec 3 and 13 item 1: OBSERVE must perturb nothing, and that is a
         * property of the CONTROLLER, not of the application. In
         * TWAI_MODE_NORMAL the peripheral ACKs every frame it receives and can
         * emit error frames, so an "application-silent" OBSERVE is still an
         * active node on a live VCM-to-GENE bus. Force TWAI_MODE_LISTEN_ONLY
         * instead of relying on can_mode being configured to silent.
         *
         * can_set_silent() only takes effect while OFF_BUS, so a controller
         * already up in the wrong mode has to be torn down first. can_disable()
         * quiesces this worker itself, so that is safe from here.
         *
         * Two consequences worth stating. The diag frames of section 10 cannot
         * go out in OBSERVE -- twai_transmit fails in listen-only -- which is
         * the documented cost of a genuinely passive tap. And we undo ONLY our
         * own forcing when leaving OBSERVE: a user who configured can_mode as
         * silent keeps that, rather than having us quietly hand them a
         * transmitting device.
         */
        const bool want_listen_only = (mode == GEN_INHIBIT_OBSERVE);

        if (want_listen_only && can_is_enabled() && !can_is_silent())
        {
            ESP_LOGW(TAG, "OBSERVE: re-opening bus listen-only");
            can_disable();
        }
        else if (!want_listen_only && s_forced_silent && can_is_enabled())
        {
            /* Leaving OBSERVE for a mode that must transmit. */
            ESP_LOGW(TAG, "leaving listen-only for mode %d", (int)mode);
            can_disable();
        }

        if (!can_is_enabled())
        {
            if (!s_forced_silent)
            {
                s_silent_saved = can_is_silent();
            }
            can_set_silent(want_listen_only ? 1 : s_silent_saved);
            s_forced_silent = want_listen_only;
            can_enable();
            s_we_enabled_bus = true;
        }
        if (!can_is_enabled())
        {
            ESP_LOGE(TAG, "cannot arm: CAN bus would not come up");
            return ESP_ERR_INVALID_STATE;
        }
        /*
         * Refuse to arm OBSERVE unless the flag that selects g_config_silent
         * is actually set. Note what this does and does not prove: the TWAI
         * API exposes no read-back of the controller's mode, so this confirms
         * the input to twai_driver_install() rather than interrogating the
         * peripheral. What makes it sound is that the driver is (re)installed
         * above, in this function, immediately after the flag is written --
         * so the flag and the installed config cannot disagree. A stale
         * already-up driver, which is exactly the case item 1 is about, is
         * torn down rather than trusted.
         */
        if (want_listen_only && !can_is_silent())
        {
            ESP_LOGE(TAG, "cannot arm OBSERVE: not in listen-only");
            return ESP_ERR_INVALID_STATE;
        }
    }

    s_offset_us = offset_us;
    /*
     * Reset only when arming. Clearing on disarm too meant a client that did
     * the obvious thing -- stop, then read the results -- got zeros back, which
     * is indistinguishable from "the device received nothing". Statistics now
     * survive the disarm and live until the next arm.
     */
    if (mode != GEN_INHIBIT_OFF)
    {
        gen_inhibit_reset_stats();
    }
    s_mode = mode;
    ESP_LOGW(TAG, "mode -> %d, probe offset %lu us", (int)mode, (unsigned long)offset_us);
    if (mode == GEN_INHIBIT_INHIBIT)
    {
        ESP_LOGW(TAG, "INHIBIT ARMED -- transmitting real 0x%03X (zero torque). "
                      "Bench only.", GEN_INHIBIT_VCM_ID);
    }

    /*
     * Releasing the bus is left to the worker, deliberately. can_disable()
     * calls twai_driver_uninstall(), and tearing the driver down from this
     * task while the worker is parked inside twai_receive() is a use-after-
     * free on the driver's internals. The worker disarms itself instead, from
     * a point where it is provably not inside the driver.
     */
    if (mode == GEN_INHIBIT_OFF && s_we_enabled_bus)
    {
        s_release_bus = true;
    }
    return ESP_OK;
}

void gen_inhibit_quiesce(void)
{
    /*
     * Two cases:
     *
     *   - Called from the worker itself (its own release path calls
     *     can_disable): it is already at its safe top-of-loop point, so just
     *     make sure it will not re-enter the driver, and return. Blocking here
     *     would deadlock -- the worker is the thing that sets s_parked.
     *
     *   - Called from any other task: force mode OFF, relinquish ownership so
     *     the caller owns the teardown, and block until the worker confirms it
     *     is outside twai_receive(). Bounded by the 200 ms receive timeout; we
     *     allow up to 1 s.
     */
    if (s_worker == NULL)
    {
        return;
    }
    if (xTaskGetCurrentTaskHandle() == s_worker)
    {
        s_mode = GEN_INHIBIT_OFF;
        return;
    }

    s_mode = GEN_INHIBIT_OFF;
    s_we_enabled_bus = false;   /* caller owns the teardown now, not us */
    s_release_bus = false;
    s_quiesce = true;

    for (int i = 0; i < 100 && !s_parked; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_parked)
    {
        ESP_LOGE(TAG, "quiesce timed out; worker did not confirm parked");
    }
}

static void send_probe(int64_t t_rx)
{
    twai_message_t tx = { 0 };
    tx.identifier = GEN_INHIBIT_PROBE_ID;
    tx.data_length_code = 8;

    /* Echo the RX timestamp so a capture can be aligned without guessing. */
    uint32_t lo = (uint32_t)t_rx;
    tx.data[0] = (uint8_t)(lo);
    tx.data[1] = (uint8_t)(lo >> 8);
    tx.data[2] = (uint8_t)(lo >> 16);
    tx.data[3] = (uint8_t)(lo >> 24);
    tx.data[4] = (uint8_t)(s_offset_us);
    tx.data[5] = (uint8_t)(s_offset_us >> 8);
    tx.data[6] = s_last_ctr;
    tx.data[7] = 0x5A;

    /*
     * Busy-wait rather than vTaskDelay: the tick is 1 ms and we are aiming at
     * sub-millisecond placement, so a delay call cannot express the target.
     * At priority 18 this spins for at most `offset_us`.
     */
    int64_t due = t_rx + (int64_t)s_offset_us;
    while (esp_timer_get_time() < due)
    {
        /* spin */
    }

    esp_err_t err = twai_transmit(&tx, 0);
    int64_t t_tx = esp_timer_get_time();

    if (err == ESP_OK)
    {
        s_tx_ok++;
        hist_add(&s_response, (uint32_t)(t_tx - t_rx));
    }
    else
    {
        s_tx_fail++;
    }
}

static void send_inhibit(const twai_message_t *rx, int64_t t_rx)
{
    /*
     * The truck-validated inhibit frame (see notes: DBC + inhibit.py + 18k
     * decoded frames). Rebuilt from the VCM frame we just received:
     *   B0     HELD     0x08, the engine-off state (spec 4.1; was mirrored)
     *   B1,B2  00 80    gen_torque_cmd = 0 (raw 0x8000, 16-bit LE, offset -32768)
     *   B3,B4  mirror   gen_rpm_ref
     *   B5     ctr+1    steal the counter the VCM is about to use (hi nibble 0)
     * DLC 6. B3/B4 and the stolen counter still make this byte-identical to the
     * VCM's next genuine frame whenever the VCM is also in 0x08, which is what
     * makes a counter-validating GENE MCU reject the VCM's real command as a
     * duplicate and accept only our zero. When the VCM asks for a start it
     * sends 0x0B and we no longer follow it there -- see the B0 note below.
     */
    if (rx->data_length_code < 6)
    {
        s_tx_fail++;
        return;
    }

    twai_message_t tx = { 0 };
    tx.identifier = GEN_INHIBIT_VCM_ID;   /* 0x051 -- the real command ID */
    tx.data_length_code = 6;
    /*
     * B0 is HELD at the engine-off state, not mirrored (spec 4.1, changed
     * 2026-09-19). Mirroring meant that when the VCM asked for a start it sent
     * 0x0B, we echoed 0x0B with zero torque, and the inverter drew ~10x its
     * engine-off power for the whole inhibit: gen_power median -0.11 kW with
     * excursions to -2.89 kW, against -0.01 kW engine-off. The VCM itself never
     * sustains (0x0B, zero torque) -- the longest such run in the corpus is
     * 0.19 s, a transition it passes through -- so the only sustained instance
     * of that combination anywhere was our own inhibit.
     *
     * Never transmit 0x10: that value is the VCM commanding generator
     * shutdown, and asserting it is the opposite of what this does. Holding a
     * constant 0x08 makes that structurally impossible here.
     */
    tx.data[0] = GEN_INHIBIT_B0_ENGINE_OFF;
    tx.data[1] = 0x00;
    tx.data[2] = 0x80;
    tx.data[3] = rx->data[3];
    tx.data[4] = rx->data[4];
    tx.data[5] = (uint8_t)(((rx->data[5] & 0x0F) + 1) & 0x0F);

    /* Reactive trail: transmit immediately, no offset wait. */
    esp_err_t err = twai_transmit(&tx, 0);
    int64_t t_tx = esp_timer_get_time();

    if (err == ESP_OK)
    {
        s_tx_ok++;
        hist_add(&s_response, (uint32_t)(t_tx - t_rx));
    }
    else
    {
        /*
         * Spec 7: abort immediately on a failed transmit. Unlike an error
         * frame, which is a property of the bus and is therefore judged on a
         * rate, a failed transmit is unambiguously OURS -- our frame did not
         * go out, so the VCM's command stands and we are not inhibiting
         * anything. There is nothing to average over.
         */
        s_tx_fail++;
        inhibit_abort("transmit failed");
    }
}

static void disable_monitor(const twai_message_t *rx)
{
    if (s_disabled)
    {
        return;   /* latched until reboot; nothing more to evaluate */
    }

    if (rx->identifier == GEN_INHIBIT_MMODE_ID && rx->data_length_code >= 7)
    {
        uint8_t pos = (rx->data[6] >> 4) & 0x07;
        s_last_shift_pos = pos;
        if (pos == GEN_INHIBIT_MMODE_VALUE)
        {
            s_disabled = true;
            s_disable_reason = "m_mode";
            s_disable_code = 1;
            ESP_LOGW(TAG, "INHIBIT DISABLED (latched): M mode engaged");
        }
    }
    else if (rx->identifier == GEN_INHIBIT_SOC_ID && rx->data_length_code >= 2)
    {
        uint32_t raw = ((uint32_t)rx->data[0] << 6) | (rx->data[1] >> 2);
        if (raw == 0)
        {
            /* Startup sentinel, not 0% -- treat as not-yet-valid. */
            s_soc_low_count = 0;
        }
        else
        {
            s_soc_raw = raw;
            if (raw < GEN_INHIBIT_SOC_MIN_RAW)
            {
                if (++s_soc_low_count >= GEN_INHIBIT_SOC_DEBOUNCE)
                {
                    s_disabled = true;
                    s_disable_reason = "low_soc";
                    s_disable_code = 2;
                    ESP_LOGW(TAG, "INHIBIT DISABLED (latched): SoC %lu.%02lu%% below %d%%",
                             (unsigned long)(raw / 100), (unsigned long)(raw % 100),
                             GEN_INHIBIT_SOC_MIN_PCT);
                }
            }
            else
            {
                s_soc_low_count = 0;
            }
        }
    }
}

/* The bus's centred 16-bit encoding: little-endian, zero at `zero`. */
static int32_t le16c(const uint8_t *d, int32_t zero)
{
    return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8)) - zero;
}

static bool fresh(int64_t stamp, int64_t now)
{
    return stamp != 0 && (now - stamp) < GEN_INHIBIT_FRESH_US;
}

/*
 * Record every interlock signal's value AND the time it was last seen. Runs on
 * every frame whatever the mode, so the gate has history the instant a mode
 * change asks for it, rather than having to wait for a fresh round after the
 * request.
 */
static void interlock_monitor(const twai_message_t *rx, int64_t now)
{
    switch (rx->identifier)
    {
    case GEN_INHIBIT_VCM_ID:
        s_seen_cmd = now;
        if (rx->data_length_code >= 5)
        {
            s_vcm_torque  = le16c(&rx->data[1], GEN_INHIBIT_CMD_ZERO);
            s_vcm_rpm_ref = le16c(&rx->data[3], GEN_INHIBIT_CMD_ZERO);
        }
        break;
    case GEN_INHIBIT_FB_ID:
        /* Liveness only. The value is not used; that it arrives is the test. */
        s_seen_fb = now;
        s_fb_ever = true;
        break;
    case GEN_INHIBIT_RPM_ID:
        s_seen_rpm = now;
        if (rx->data_length_code >= 2)
        {
            s_gene_rpm = le16c(&rx->data[0], GEN_INHIBIT_RPM_ZERO);
        }
        break;
    case GEN_INHIBIT_FAULT_ID:
        if (rx->data_length_code >= 8)
        {
            s_vcm_fault = rx->data[7];
        }
        break;
    default:
        break;
    }
}

static void inhibit_abort(const char *why)
{
    s_abort_reason = why;
    s_arm_block = why;
    s_inhibit_live = false;
    s_mode = GEN_INHIBIT_OFF;
    ESP_LOGE(TAG, "INHIBIT ABORT: %s", why);
}

/*
 * Spec 7: may the inhibit go live right now? Every check is against FRESHNESS
 * as well as value -- a silent bus otherwise answers every question you ask
 * it, and answers them all reassuringly.
 */
static bool arm_gate_ok(int64_t now)
{
    if (!fresh(s_seen_cmd, now))  { s_arm_block = "no fresh 0x051";        return false; }
    /*
     * 0x471 and 0x054 are deliberately NOT required here, and that is a
     * correction (2026-09-19) rather than an omission.
     *
     * Measured across bus wake events in the corpus: after the bus comes
     * alive, 0x051 appears within ~0.17 s, but the GENE family (0x471, 0x054)
     * appears +28 s later, or not at all within 60 s -- the inverter powers up
     * long after the VCM, and in sessions where the generator is never used it
     * never appears. Requiring them to arm would mean the inhibit went live
     * ~28 s after key-on at best and never at worst, while the evap-driven
     * start it exists to prevent follows key-on almost immediately.
     *
     * It is also self-defeating: requiring fresh 0x471 means arming only once
     * the inverter is powered, which is the neighbourhood of the state spec 7
     * explicitly refuses to arm into. Their absence is positive evidence the
     * generator is NOT running.
     *
     * Spec 7 lists 0x471 only as a DISARM condition -- "disarm if the inverter
     * goes quiet (0x471 stops)" -- and "stops" presupposes it was going. The
     * runtime trip below honours that by firing only once it has been seen.
     */
    if (s_vcm_fault == GEN_INHIBIT_FAULT_ACTIVE)
                                  { s_arm_block = "VCM fault active";      return false; }
    /*
     * Refuse while the generator is already running or the VCM is asking for
     * torque. Taking over a loaded generator and commanding zero sheds the
     * engine's whole load in one frame -- a load dump on a running engine, and
     * the wrong way to find out whether the mechanism works. The only
     * transition this arms into is "engine off, VCM tries to start it, we stop
     * it".
     */
    /*
     * Only meaningful if 0x054 is actually arriving. A stale reading cannot
     * say the generator is running -- but nor can it say it is stopped, so
     * this leans on the GENE family's silence being evidence of an unpowered
     * inverter rather than trusting a last-known value.
     */
    if (fresh(s_seen_rpm, now) && s_gene_rpm >= GEN_INHIBIT_START_ABORT_RPM)
                                  { s_arm_block = "generator running";     return false; }
    if (s_vcm_rpm_ref >= 0)       { s_arm_block = "VCM requesting engine"; return false; }
    if (s_vcm_torque != 0)        { s_arm_block = "VCM commanding torque"; return false; }
    if (s_shutdown_suppressed)    { s_arm_block = "VCM commanding 0x10";   return false; }
    s_arm_block = "";
    return true;
}

/* Spec 7: the trips that end a live run. Called every loop, not only on an
 * 0x051, so a bus that goes quiet is caught by the 200 ms receive timeout. */
static void interlock_runtime(int64_t now)
{
    if (s_vcm_fault == GEN_INHIBIT_FAULT_ACTIVE)
    {
        inhibit_abort("0x617 B7 = 0xCA (VCM fault active)");
        return;
    }

    /*
     * THE BUS ITSELF HAS GONE. 0x051 runs at ~100 Hz whenever the link is up,
     * so its absence is the anchor test for "we are no longer connected to the
     * powertrain bus" -- a pulled connector, a dropped transceiver, key-off.
     * Framed as link loss deliberately, rather than as individual signals
     * going offline: the realistic failure is the CAN connection dropping
     * entirely, in which case every signal stops together and 0x051 is the one
     * to notice it by.
     *
     * THIS LATCHES. Decided 2026-09-19. The tempting alternative is to stand
     * down and re-arm when the bus returns, on the grounds that link loss is
     * an absence rather than a fault and a momentary glitch should not cost a
     * drive. That is rejected: if the link is INTERMITTENT, resuming is the
     * wrong response. A bus that drops and returns is an unreliable
     * environment, and this device does not merely observe it -- it steals the
     * VCM's rolling counter and transmits a real 0x051 onto a live powertrain
     * bus. Coming back automatically would mean doing that repeatedly across a
     * link we already have evidence is unsound, and each resume would re-enter
     * through a gate whose freshness checks a flapping bus can satisfy.
     * Latching turns an intermittent connection into one clean stand-down
     * instead of an oscillation.
     *
     * Latched like the section 6 releases: cleared only by a reboot, which the
     * relay dropping at truck sleep provides. A consequence worth knowing is
     * that a key-off/key-on quick enough that the relay never opens leaves the
     * inhibit latched off for that second drive -- which is the intended
     * reading of "do not resume", not an oversight.
     *
     * THIS REPLACES THE DEAD-MAN TIMER, which is removed (spec 7, 2026-09-19).
     * Every hazard the timer was still covering reduces to "the signals that
     * would release us stopped arriving", and losing the VCU's 0x051 and the
     * BCM's 0x411 together has no plausible cause on a healthy link -- it IS
     * link loss, which this catches directly and by evidence rather than by
     * elapsed time. Section 6 handles the cases where the truck legitimately
     * wants the engine; this handles the case where we can no longer see the
     * truck at all. A timer on top would only add a way for the inhibit to
     * stop mid-drive for no observed reason.
     */
    if (!fresh(s_seen_cmd, now))
    {
        inhibit_abort("bus lost -- no 0x051 (latched; CAN link down)");
        return;
    }

    /*
     * "Disarm if the inverter goes QUIET (0x471 STOPS)" -- so this can only
     * fire once 0x471 has actually been heard. Without s_fb_ever the trip
     * would fire the instant we went live on a bus whose inverter has not
     * powered up yet, which measurement says is the normal case for the first
     * ~28 s after key-on and is permanent in sessions where the generator is
     * never used. Never-seen is not the same as stopped.
     *
     * 0x051 is known fresh by this point, so this is unambiguously the
     * inverter specifically dropping off a live bus -- the alarming case, and
     * the distinction the laptop tool could not make. Whole-bus loss was
     * handled above and is not a fault.
     */
    if (s_fb_ever && !fresh(s_seen_fb, now))
    {
        inhibit_abort("0x471 stopped while 0x051 still live -- inverter lost");
        return;
    }

    if (s_gene_rpm >= GEN_INHIBIT_START_ABORT_RPM)
    {
        if (s_rpm_over_since == 0)
        {
            s_rpm_over_since = now;
        }
        else if (now - s_rpm_over_since > GEN_INHIBIT_RPM_DEBOUNCE_US)
        {
            inhibit_abort("engine turning while armed -- the inhibit did not hold");
            return;
        }
    }
    else
    {
        s_rpm_over_since = 0;
    }

    /*
     * Error frames, rate-based over a sliding window. Note this counts the
     * controller's own bus_error_count rather than anything payload-derived,
     * so it is distinct from the failed-transmit trip in send_inhibit(), which
     * is unambiguously about a frame of ours.
     */
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK)
    {
        if (s_err_window == 0 || (now - s_err_window) > GEN_INHIBIT_ERR_WINDOW_US)
        {
            s_err_window = now;
            s_err_base = st.bus_error_count;
        }
        else if ((uint32_t)(st.bus_error_count - s_err_base)
                 >= GEN_INHIBIT_ERR_MIN_TRIP)
        {
            inhibit_abort("error-frame rate exceeded");
            return;
        }
    }
}

static uint32_t fnv1a32(const char *s)
{
    uint32_t h = 2166136261u;
    while (*s)
    {
        h ^= (uint8_t)*s++;
        h *= 16777619u;
    }
    return h;
}

static void send_diag(uint8_t which)
{
    twai_message_t tx = { 0 };
    tx.data_length_code = 8;

    if (which == 0)   /* STATUS -> 0x7F1 */
    {
        uint8_t flags = (s_disabled ? 0x01 : 0)
                      | (can_is_enabled() ? 0x02 : 0)
                      | (s_we_enabled_bus ? 0x04 : 0)
                      | ((GEN_INHIBIT_AUTOARM_MODE != GEN_INHIBIT_OFF) ? 0x08 : 0)
                      /*
                       * bit4: live, non-latching. Spec 6.4 puts this in the
                       * flags rather than disable_reason precisely because it
                       * comes and goes -- a reason code that appeared and
                       * vanished as the truck parked and woke could not be
                       * told apart from a past one when reading a log.
                       */
                      | (s_shutdown_suppressed ? 0x10 : 0)
                      /* bit5: spec 7 interlocks satisfied and transmitting.
                       * Distinct from mode 3, which is only what was asked
                       * for -- the two differ for the whole time the gate is
                       * waiting on the bus, and that gap is the interesting
                       * thing to see in a log. */
                      | (s_inhibit_live ? 0x20 : 0);
        tx.identifier = GEN_INHIBIT_DIAG_ID_STATUS;
        tx.data[0] = GEN_INHIBIT_DIAG_SCHEMA_VER;
        tx.data[1] = (uint8_t)DIAG_FW_VERSION;
        tx.data[2] = (uint8_t)s_mode;
        tx.data[3] = flags;
        tx.data[4] = s_disable_code;
        tx.data[5] = s_last_shift_pos;
        tx.data[6] = (uint8_t)s_soc_raw;         /* soc_x100, LE */
        tx.data[7] = (uint8_t)(s_soc_raw >> 8);
    }
    else if (which == 1)   /* COUNTERS -> 0x7F2 */
    {
        uint32_t v = s_tx_ok;
        tx.identifier = GEN_INHIBIT_DIAG_ID_COUNTERS;
        tx.data[0] = (uint8_t)v;
        tx.data[1] = (uint8_t)(v >> 8);
        tx.data[2] = (uint8_t)(v >> 16);
        tx.data[3] = (uint8_t)(v >> 24);
        tx.data[4] = s_tx_fail       > 255 ? 255 : (uint8_t)s_tx_fail;
        tx.data[5] = s_ctr_steps_bad > 255 ? 255 : (uint8_t)s_ctr_steps_bad;
        tx.data[6] = s_rx_errors     > 255 ? 255 : (uint8_t)s_rx_errors;
    }
    else   /* BUILD -> 0x7F3 */
    {
        uint32_t h = s_git_hash;
        tx.identifier = GEN_INHIBIT_DIAG_ID_BUILD;
        tx.data[0] = (uint8_t)h;
        tx.data[1] = (uint8_t)(h >> 8);
        tx.data[2] = (uint8_t)(h >> 16);
        tx.data[3] = (uint8_t)(h >> 24);
        tx.data[4] = (uint8_t)(DIAG_FW_VERSION);
        tx.data[5] = (uint8_t)(DIAG_FW_VERSION >> 8);
        tx.data[6] = GEN_INHIBIT_DIAG_SCHEMA_VER;
    }

    twai_transmit(&tx, 0);   /* best-effort; fails silently in listen-only */
}

static void gen_inhibit_task(void *arg)
{
    static int64_t t_prev_051 = 0;
    static int64_t t_last_diag = 0;
    static uint8_t diag_page = 0;
    twai_message_t rx;

    s_worker = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "worker started (prio %d)", GEN_INHIBIT_TASK_PRIO);

    while (1)
    {
        if (s_mode == GEN_INHIBIT_OFF)
        {
            /*
             * Clear the live flag on EVERY route into OFF, not just on the
             * abort path. gen_inhibit_reset_stats() only runs when arming, so
             * an ordinary disarm would otherwise leave s_inhibit_live set:
             * harmless for transmission, since the dispatch also tests the
             * mode, but it would make diag_flags bit5 and the JSON claim a
             * live inhibit on a disarmed device -- exactly the kind of
             * reassuring-but-wrong reading these flags exist to prevent.
             */
            if (s_inhibit_live)
            {
                s_inhibit_live = false;
                s_arm_block = "not armed";
            }
            /*
             * Safe teardown point: we are not inside twai_receive() here, so
             * uninstalling the driver cannot pull the floor out from under a
             * blocked call. Publish that fact so an external gen_inhibit_quiesce()
             * caller (sleep monitor, SLCAN close) knows it can now uninstall.
             */
            s_parked = true;
            /*
             * Unconditionally, not just on release: a stale t_prev_051 would
             * make the first gap after the next arm the whole idle interval,
             * silently poisoning every run after the first.
             */
            t_prev_051 = 0;

            if (s_release_bus)
            {
                s_release_bus = false;
                s_we_enabled_bus = false;
                can_disable();
                /*
                 * Undo an OBSERVE listen-only forcing now that the driver is
                 * down -- can_set_silent() is a no-op while ON_BUS, so this
                 * has to happen after can_disable() and not before it.
                 */
                if (s_forced_silent)
                {
                    can_set_silent(s_silent_saved);
                    s_forced_silent = false;
                }
                ESP_LOGW(TAG, "bus released");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /*
         * Blocking receive: the TWAI ISR wakes us directly, so this costs the
         * ISR-to-task latency rather than wican-fw's 1 ms poll interval. That
         * difference is the whole reason this component exists.
         */
        /*
         * Self-telemetry heartbeat, round-robin across the three diag IDs. Runs
         * before the blocking receive so it still fires on a quiet bus (the
         * 200 ms timeout bounds the gap). Armed only -- past the OFF branch.
         */
        int64_t now = esp_timer_get_time();
        if (now - t_last_diag >= (int64_t)GEN_INHIBIT_DIAG_PERIOD_MS * 1000)
        {
            t_last_diag = now;
            send_diag(diag_page);
            diag_page = (diag_page + 1) % 3;
        }

        /*
         * Spec 7 interlocks. Evaluated here, before the blocking receive, so
         * they still fire on a bus that has gone silent -- which is precisely
         * the case the 0x471 and bus-loss trips exist for. A trip that only
         * ran on frame arrival could not detect the absence of frames.
         *
         * LOAD-BEARING INVARIANT: interlock_runtime() runs ONLY once the
         * inhibit is live, never on the path to going live. The bus-loss trip
         * inside it latches, and before key-on there is legitimately no 0x051
         * at all (measured: the PT bus is silent until the key turns, and the
         * GENE family for ~28 s after that). Running the trips on a
         * not-yet-live inhibit would therefore latch the device off during the
         * ordinary pre-key-on wait -- on an auto-arm build, before it had ever
         * inhibited anything. Absence of the bus BEFORE going live is
         * "not ready yet", handled by arm_gate_ok(); absence AFTER going live
         * is "the link we were using has dropped". Do not merge these paths.
         */
        if (s_mode == GEN_INHIBIT_INHIBIT && !s_disabled)
        {
            if (s_inhibit_live)
            {
                interlock_runtime(now);
            }
            else if (arm_gate_ok(now))
            {
                s_inhibit_live = true;
                s_rpm_over_since = 0;
                s_err_window = 0;
                s_abort_reason = "";
                ESP_LOGW(TAG, "INHIBIT LIVE -- interlocks passed");
            }
        }

        /*
         * 200 ms rather than a full second only so a disarm is acted on
         * promptly; a timeout costs nothing but a loop iteration.
         */
        s_parked = false;   /* about to enter the driver */
        esp_err_t rx_err = twai_receive(&rx, pdMS_TO_TICKS(200));
        if (rx_err != ESP_OK)
        {
            /*
             * ESP_ERR_TIMEOUT is ordinary: it just means a quiet second on the
             * bus, and the call already blocked for it. Anything else returns
             * *immediately* -- ESP_ERR_INVALID_STATE when the driver is not
             * installed -- so retrying without a delay is a busy-wait at
             * priority 18 that starves every lower-priority task, httpd
             * included. Always yield on the error path.
             */
            if (rx_err == ESP_ERR_TIMEOUT)
            {
                continue;
            }

            s_rx_errors++;
            if (s_rx_errors == 1)
            {
                ESP_LOGE(TAG, "twai_receive failed: %s", esp_err_to_name(rx_err));
            }
            vTaskDelay(pdMS_TO_TICKS(100));

            if (s_rx_errors >= GEN_INHIBIT_MAX_RX_ERRORS)
            {
                ESP_LOGE(TAG, "disarming after %lu receive errors",
                         (unsigned long)s_rx_errors);
                s_mode = GEN_INHIBIT_OFF;
                s_rx_errors = 0;
            }
            continue;
        }
        s_rx_errors = 0;

        int64_t t_rx = esp_timer_get_time();

        /* Watch for M mode / low SoC on every frame -- latches the disable. */
        disable_monitor(&rx);

        /*
         * Spec 7: freshness and values for the interlock signals. Must run
         * before the 0x051 filter below -- 0x471, 0x054 and 0x617 are all
         * "other frames" as far as the inhibit is concerned, and they are
         * exactly the ones the safety trips depend on.
         */
        interlock_monitor(&rx, t_rx);

        if (rx.identifier != GEN_INHIBIT_VCM_ID)
        {
            s_other_frames++;
            continue;
        }

        if (t_prev_051 != 0)
        {
            hist_add(&s_rx_gap, (uint32_t)(t_rx - t_prev_051));
        }
        t_prev_051 = t_rx;

        /*
         * Spec 6.3: track the VCM's shutdown command live, on every 0x051.
         * Recomputed from the current frame rather than latched, so leaving
         * 0x10 resumes transmission by itself with no clearing step. Logged on
         * the edge only -- an episode can last 561 s and a per-frame log at
         * 100 Hz would bury everything else.
         */
        if (rx.data_length_code >= 1)
        {
            bool now_shutdown = (rx.data[0] == GEN_INHIBIT_B0_SHUTDOWN);
            if (now_shutdown != s_shutdown_suppressed)
            {
                s_shutdown_suppressed = now_shutdown;
                ESP_LOGW(TAG, "shutdown suppression %s (0x051 B0 = 0x%02X)",
                         now_shutdown ? "ON" : "OFF", rx.data[0]);
            }
        }

        /*
         * B5 is the VCM's rolling counter -- 6422 of 6422 steps were exactly
         * +1 in the reference capture. Tracking it here is a cheap check that
         * we are seeing every frame rather than silently dropping some.
         */
        if (rx.data_length_code >= 6)
        {
            uint8_t ctr = rx.data[5] & 0x0F;
            if (s_have_last_ctr)
            {
                if (((s_last_ctr + 1) & 0x0F) == ctr) s_ctr_steps_ok++;
                else                                  s_ctr_steps_bad++;
            }
            s_last_ctr = ctr;
            s_have_last_ctr = true;
        }

        if (s_mode == GEN_INHIBIT_RESPOND)
        {
            send_probe(t_rx);
        }
        else if (s_mode == GEN_INHIBIT_INHIBIT && !s_disabled
                 && !s_shutdown_suppressed && s_inhibit_live)
        {
            send_inhibit(&rx, t_rx);
        }
    }
}

int gen_inhibit_get_stats_json(char *buf, int buflen)
{
    int n = snprintf(buf, buflen,
                     "{\"mode\":%d,\"offset_us\":%lu,\"probe_id\":\"0x%03X\","
                     "\"tx_ok\":%lu,\"tx_fail\":%lu,\"other_frames\":%lu,"
                     "\"ctr_ok\":%lu,\"ctr_bad\":%lu,"
                     "\"bus_on\":%s,\"bus_ours\":%s,\"rx_errors\":%lu,"
                     "\"disabled\":%s,\"disable_reason\":\"%s\",\"soc_x100\":%lu,"
                     "\"shutdown_suppressed\":%s,"
                     "\"inhibit_live\":%s,\"arm_block\":\"%s\","
                     "\"abort_reason\":\"%s\",\"gene_rpm\":%ld,"
                     "\"vcm_torque\":%ld,\"vcm_fault\":%u,",
                     (int)s_mode, (unsigned long)s_offset_us, GEN_INHIBIT_PROBE_ID,
                     (unsigned long)s_tx_ok, (unsigned long)s_tx_fail,
                     (unsigned long)s_other_frames,
                     (unsigned long)s_ctr_steps_ok, (unsigned long)s_ctr_steps_bad,
                     can_is_enabled() ? "true" : "false",
                     s_we_enabled_bus ? "true" : "false",
                     (unsigned long)s_rx_errors,
                     s_disabled ? "true" : "false", s_disable_reason,
                     (unsigned long)s_soc_raw,
                     s_shutdown_suppressed ? "true" : "false",
                     s_inhibit_live ? "true" : "false", s_arm_block,
                     s_abort_reason, (long)s_gene_rpm,
                     (long)s_vcm_torque, (unsigned)s_vcm_fault);

    n += hist_json(&s_rx_gap, "rx_gap", buf + n, buflen - n);
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",");
    n += hist_json(&s_response, "response", buf + n, buflen - n);
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",\"bucket_edges_us\":[");
    for (int i = 0; i < NBUCKETS - 1 && n < buflen; i++)
    {
        n += snprintf(buf + n, buflen - n, "%s%lu", (i ? "," : ""),
                      (unsigned long)s_bucket_us[i]);
    }
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",\"inf\"]}\n");
    return n;
}

void gen_inhibit_init(void)
{
    s_git_hash = fnv1a32(GIT_SHA);
    gen_inhibit_reset_stats();
    xTaskCreate(gen_inhibit_task, "gen_inhibit", GEN_INHIBIT_STACK, NULL,
                GEN_INHIBIT_TASK_PRIO, NULL);

    if (GEN_INHIBIT_AUTOARM_MODE != GEN_INHIBIT_OFF)
    {
        ESP_LOGW(TAG, "auto-arm on boot: mode %d", (int)GEN_INHIBIT_AUTOARM_MODE);
        gen_inhibit_set_mode(GEN_INHIBIT_AUTOARM_MODE, GEN_INHIBIT_AUTOARM_OFFSET_US);
    }
}
