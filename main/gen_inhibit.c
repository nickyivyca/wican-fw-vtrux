/*
 * gen_inhibit -- driver shim around gen_inhibit_core.
 *
 * SPLIT (2026-09-19). The decision logic moved to gen_inhibit_core.c, which
 * has no platform dependency and is compiled by a host test harness. What is
 * left here is everything that talks to the ESP32: the worker task, the TWAI
 * driver, the bus enable/disable dance, logging, and the JSON view.
 *
 * The rule for future edits: if a change is about WHAT to decide, it belongs
 * in the core and gains host test coverage for free. If it is about HOW to
 * receive, transmit, or wait, it belongs here and is only ever proved on
 * hardware. When in doubt, put it in the core -- the core is the part that
 * three real bugs were found in by hand on the day this split was written.
 */
#include "gen_inhibit.h"
#include "gen_inhibit_core.h"

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
 * The public mode enum and the core's must agree numerically -- the shim
 * casts between them rather than translating. Asserted, not commented.
 */
_Static_assert((int)GEN_INHIBIT_OFF     == (int)GI_OFF,     "mode enum drift");
_Static_assert((int)GEN_INHIBIT_OBSERVE == (int)GI_OBSERVE, "mode enum drift");
_Static_assert((int)GEN_INHIBIT_RESPOND == (int)GI_RESPOND, "mode enum drift");
_Static_assert((int)GEN_INHIBIT_INHIBIT == (int)GI_INHIBIT, "mode enum drift");
_Static_assert(GEN_INHIBIT_VCM_ID   == GI_VCM_ID,   "id drift");
_Static_assert(GEN_INHIBIT_PROBE_ID == GI_PROBE_ID, "id drift");

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

/* Bump per meaningful firmware rev and add a row to the table in
 * projects/vtrux/notes/artifacts/gen-inhibit/wican_diag_schema.md.
 *
 * 2 is the first build in which a latched section 6 release clears
 * inhibit_live, which is visible on the wire as diag_flags byte3 0x27 -> 0x07.
 * A bench log that cannot tell rev 1 from rev 2 cannot tell whether it is
 * looking at the bug or the fix, and diag_git_hash alone puts that behind a
 * lookup. */
#ifndef DIAG_FW_VERSION
#define DIAG_FW_VERSION              2
#endif
#ifndef GIT_SHA
#define GIT_SHA "unknown"
#endif

/*
 * Build-time override for the SoC floor. Every other tunable now lives in
 * gi_config_defaults(), where its justification is; this one keeps its macro
 * because measurement builds have used it. Adding more overrides here is the
 * obvious place for an eventual over-the-wire config to land.
 */
/* #define GEN_INHIBIT_SOC_MIN_PCT 21 */

/* Receive timeout: 200 ms rather than a full second only so a disarm is
 * acted on promptly; a timeout costs nothing but a loop iteration. */
#define GEN_INHIBIT_RX_TIMEOUT_MS   200

/* ------------------------------------------------------------------------- */

/*
 * The decision state. Shared between the worker task and the HTTP handlers,
 * with the same coarse discipline the pre-split code had: `mode` is the
 * synchronisation point and everything else is read racily for reporting.
 * Unchanged by the split, and worth flagging rather than hiding -- a
 * set_mode() arriving mid-loop still resets histograms under the worker.
 */
static gi_state_t s_core;

static bool     s_we_enabled_bus;   /* we called can_enable(), so we may disable */
static volatile bool s_release_bus; /* disarm asked us to hand the bus back */

static TaskHandle_t  s_worker;      /* the worker task, for self-call detection */
static volatile bool s_quiesce;     /* an external caller needs the driver freed */
static volatile bool s_parked;      /* worker is provably outside twai_receive() */

/*
 * Spec 13 item 1: we put the controller into listen-only for OBSERVE.
 * Remember that we did, and what the configured value was, so leaving OBSERVE
 * restores the user's setting rather than asserting one of our own.
 */
static bool     s_forced_silent;
static uint8_t  s_silent_saved;

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

static void bus_snapshot(gi_bus_t *b)
{
    twai_status_info_t st;
    memset(b, 0, sizeof(*b));
    b->can_enabled = can_is_enabled();
    b->bus_ours    = s_we_enabled_bus;
    if (twai_get_status_info(&st) == ESP_OK)
    {
        b->err_valid = true;
        b->bus_error_count = st.bus_error_count;
    }
}

/*
 * Render the core's event list to the log. The core cannot log -- it has no
 * printf -- so this is where an event becomes a line. Keeping the rendering
 * here rather than in the core is what lets the host harness assert on the
 * event CODES instead of on formatted text.
 */
static void report_events(const gi_events_t *ev)
{
    for (int i = 0; i < ev->n; i++)
    {
        const gi_event_t *e = &ev->e[i];
        switch ((gi_event_kind_t)e->kind)
        {
        case GI_EV_MODE:
            ESP_LOGW(TAG, "mode -> %d, probe offset %lu us",
                     (int)e->a, (unsigned long)e->b);
            if (e->a == (int32_t)GI_INHIBIT)
            {
                ESP_LOGW(TAG, "INHIBIT ARMED -- transmitting real 0x%03X "
                              "(zero torque). Bench only.", GI_VCM_ID);
            }
            break;
        case GI_EV_DISABLED:
            if (e->a == (int32_t)GI_DISABLE_LOW_SOC)
            {
                ESP_LOGW(TAG, "INHIBIT DISABLED (latched): SoC %ld.%02ld%% "
                              "below %lu%%",
                         (long)(e->b / 100), (long)(e->b % 100),
                         (unsigned long)(s_core.cfg.soc_min_raw / 100));
            }
            else
            {
                ESP_LOGW(TAG, "INHIBIT DISABLED (latched): M mode engaged");
            }
            break;
        case GI_EV_SHUTDOWN_SUPPRESS:
            ESP_LOGW(TAG, "shutdown suppression %s (0x051 B0 = 0x%02X)",
                     e->a ? "ON" : "OFF", (unsigned)e->b);
            break;
        case GI_EV_INHIBIT_LIVE:
            ESP_LOGW(TAG, "INHIBIT LIVE -- interlocks passed");
            break;
        case GI_EV_ABORT:
            ESP_LOGE(TAG, "INHIBIT ABORT: %s", gi_abort_name((gi_abort_t)e->a));
            break;
        case GI_EV_TX_FAIL:
            /* Counted in the core; only the inhibit case is worth a line. */
            if (e->a == (int32_t)GI_TX_INHIBIT)
            {
                ESP_LOGE(TAG, "inhibit transmit failed");
            }
            break;
        case GI_EV_RX_ERROR_DISARM:
            ESP_LOGE(TAG, "disarming after %ld receive errors", (long)e->a);
            break;
        case GI_EV_NONE:
        default:
            break;
        }
    }
    if (ev->dropped)
    {
        ESP_LOGE(TAG, "event list overflow (%u dropped) -- GI_EVENT_MAX too small",
                 (unsigned)ev->dropped);
    }
}

/*
 * Put the core's emitted frames on the wire and feed the outcome back.
 *
 * The busy-wait for a probe's due time lives here, not in the core: spinning
 * is a platform behaviour, and a host harness replaying a capture must not do
 * it. vTaskDelay cannot express it either -- the tick is 1 ms and we are
 * aiming at sub-millisecond placement.
 */
static void dispatch_emits(const gi_emit_t *em, gi_events_t *ev)
{
    for (int i = 0; i < em->n; i++)
    {
        const gi_frame_t *f = &em->f[i];
        twai_message_t tx = { 0 };
        tx.identifier = f->id;
        tx.data_length_code = f->dlc;
        memcpy(tx.data, f->data, 8);

        if (f->have_due)
        {
            /* At priority 18 this spins for at most `offset_us`. */
            while (esp_timer_get_time() < f->due_us)
            {
                /* spin */
            }
        }

        esp_err_t err = twai_transmit(&tx, 0);
        if (f->kind == GI_TX_DIAG)
        {
            continue;   /* best-effort; fails silently in listen-only */
        }
        gi_on_tx_result(&s_core, f, err == ESP_OK, esp_timer_get_time(), ev);
    }
    if (em->dropped)
    {
        ESP_LOGE(TAG, "emit list overflow (%u dropped) -- GI_EMIT_MAX too small",
                 (unsigned)em->dropped);
    }
}

/* ------------------------------------------------------------ public API -- */

void gen_inhibit_reset_stats(void)
{
    gi_reset_stats(&s_core);
}

bool gen_inhibit_owns_bus(void)
{
    return s_core.mode != GI_OFF;
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

    gi_events_t ev = { 0 };
    gi_set_mode(&s_core, (gi_mode_t)mode, offset_us, esp_timer_get_time(), &ev);
    report_events(&ev);

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
     *     is outside twai_receive(). Bounded by the receive timeout; we allow
     *     up to 1 s.
     */
    if (s_worker == NULL)
    {
        return;
    }
    if (xTaskGetCurrentTaskHandle() == s_worker)
    {
        s_core.mode = GI_OFF;
        return;
    }

    s_core.mode = GI_OFF;
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

/* ---------------------------------------------------------------- worker -- */

static void gen_inhibit_task(void *arg)
{
    twai_message_t rx;

    s_worker = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "worker started (prio %d)", GEN_INHIBIT_TASK_PRIO);

    while (1)
    {
        if (s_core.mode == GI_OFF)
        {
            gi_notify_off(&s_core);
            /*
             * Safe teardown point: we are not inside twai_receive() here, so
             * uninstalling the driver cannot pull the floor out from under a
             * blocked call. Publish that fact so an external
             * gen_inhibit_quiesce() caller (sleep monitor, SLCAN close) knows
             * it can now uninstall.
             */
            s_parked = true;

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
         * The periodic half: diag heartbeat and the spec 7 interlocks. Runs
         * BEFORE the blocking receive so both still fire on a quiet bus --
         * which is precisely the case the bus-loss and 0x471 trips exist for.
         * A trip that only ran on frame arrival could not detect the absence
         * of frames.
         */
        {
            gi_bus_t bus;
            gi_emit_t em = { 0 };
            gi_events_t ev = { 0 };
            bus_snapshot(&bus);
            gi_tick(&s_core, esp_timer_get_time(), &bus, &em, &ev);
            dispatch_emits(&em, &ev);
            report_events(&ev);
        }

        /*
         * Blocking receive: the TWAI ISR wakes us directly, so this costs the
         * ISR-to-task latency rather than wican-fw's 1 ms poll interval. That
         * difference is the whole reason this component exists.
         */
        s_parked = false;   /* about to enter the driver */
        esp_err_t rx_err = twai_receive(&rx, pdMS_TO_TICKS(GEN_INHIBIT_RX_TIMEOUT_MS));
        if (rx_err != ESP_OK)
        {
            /*
             * ESP_ERR_TIMEOUT is ordinary: it just means a quiet moment on the
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

            gi_events_t ev = { 0 };
            if (s_core.rx_errors == 0)
            {
                ESP_LOGE(TAG, "twai_receive failed: %s", esp_err_to_name(rx_err));
            }
            gi_on_rx_error(&s_core, esp_timer_get_time(), &ev);
            report_events(&ev);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        gi_on_rx_ok(&s_core);

        {
            gi_emit_t em = { 0 };
            gi_events_t ev = { 0 };
            gi_on_frame(&s_core, rx.identifier, rx.data_length_code, rx.data,
                        esp_timer_get_time(), &em, &ev);
            dispatch_emits(&em, &ev);
            report_events(&ev);
        }
    }
}

/* ------------------------------------------------------------------ JSON -- */

static int hist_json(const gi_hist_t *h, const char *name, char *buf, int buflen)
{
    int n = snprintf(buf, buflen,
                     "\"%s\":{\"count\":%lu,\"min_us\":%lu,\"max_us\":%lu,\"mean_us\":%lu,\"buckets\":[",
                     name, (unsigned long)h->count,
                     (unsigned long)(h->count ? h->min_us : 0),
                     (unsigned long)h->max_us,
                     (unsigned long)(h->count ? (uint32_t)(h->sum_us / h->count) : 0));
    for (int i = 0; i < GI_NBUCKETS && n < buflen; i++)
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

int gen_inhibit_get_stats_json(char *buf, int buflen)
{
    const gi_state_t *st = &s_core;

    /*
     * arm_block reports the abort text once a run has ended, matching the
     * pre-split behaviour where inhibit_abort() wrote the same string into
     * both fields. Cleared together on the next arm.
     */
    const char *block = (st->abort_reason != GI_ABORT_NONE)
                        ? gi_abort_name(st->abort_reason)
                        : gi_block_name(st->arm_block);

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
                     (int)st->mode, (unsigned long)st->offset_us, GI_PROBE_ID,
                     (unsigned long)st->tx_ok, (unsigned long)st->tx_fail,
                     (unsigned long)st->other_frames,
                     (unsigned long)st->ctr_steps_ok,
                     (unsigned long)st->ctr_steps_bad,
                     can_is_enabled() ? "true" : "false",
                     s_we_enabled_bus ? "true" : "false",
                     (unsigned long)st->rx_errors,
                     st->disabled ? "true" : "false",
                     gi_disable_name(st->disable_code),
                     (unsigned long)st->soc_raw,
                     st->shutdown_suppressed ? "true" : "false",
                     st->inhibit_live ? "true" : "false", block,
                     gi_abort_name(st->abort_reason), (long)st->gene_rpm,
                     (long)st->vcm_torque, (unsigned)st->vcm_fault);

    n += hist_json(&st->rx_gap, "rx_gap", buf + n, buflen - n);
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",");
    n += hist_json(&st->response, "response", buf + n, buflen - n);
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",\"bucket_edges_us\":[");
    for (int i = 0; i < GI_NBUCKETS - 1 && n < buflen; i++)
    {
        n += snprintf(buf + n, buflen - n, "%s%lu", (i ? "," : ""),
                      (unsigned long)gi_bucket_us[i]);
    }
    if (n < buflen) n += snprintf(buf + n, buflen - n, ",\"inf\"]}\n");
    return n;
}

/* ------------------------------------------------------------------ init -- */

void gen_inhibit_init(void)
{
    gi_config_t cfg;
    gi_config_defaults(&cfg);
#ifdef GEN_INHIBIT_SOC_MIN_PCT
    cfg.soc_min_raw = GEN_INHIBIT_SOC_MIN_PCT * 100;
#endif
    cfg.fw_version = DIAG_FW_VERSION;
    cfg.git_hash   = fnv1a32(GIT_SHA);
    cfg.autoarm_configured = (GEN_INHIBIT_AUTOARM_MODE != GEN_INHIBIT_OFF);

    gi_init(&s_core, &cfg);

    xTaskCreate(gen_inhibit_task, "gen_inhibit", GEN_INHIBIT_STACK, NULL,
                GEN_INHIBIT_TASK_PRIO, NULL);

    if (GEN_INHIBIT_AUTOARM_MODE != GEN_INHIBIT_OFF)
    {
        ESP_LOGW(TAG, "auto-arm on boot: mode %d", (int)GEN_INHIBIT_AUTOARM_MODE);
        gen_inhibit_set_mode(GEN_INHIBIT_AUTOARM_MODE, GEN_INHIBIT_AUTOARM_OFFSET_US);
    }
}
