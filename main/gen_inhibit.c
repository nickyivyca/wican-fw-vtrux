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
#define GEN_INHIBIT_SOC_MIN_PCT     20
#endif
#define GEN_INHIBIT_SOC_MIN_RAW     (GEN_INHIBIT_SOC_MIN_PCT * 100)
#define GEN_INHIBIT_SOC_DEBOUNCE    5

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
static uint32_t      s_soc_raw;         /* last valid SoC, raw (percent*100) */
static uint32_t      s_soc_low_count;   /* consecutive valid sub-threshold samples */

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
        if (!can_is_enabled())
        {
            can_enable();
            s_we_enabled_bus = true;
        }
        if (!can_is_enabled())
        {
            ESP_LOGE(TAG, "cannot arm: CAN bus would not come up");
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
     *   B0     mirror   (state: 0x08 idle / 0x0b start-requested)
     *   B1,B2  00 80    gen_torque_cmd = 0 (raw 0x8000, 16-bit LE, offset -32768)
     *   B3,B4  mirror   gen_rpm_ref
     *   B5     ctr+1    steal the counter the VCM is about to use (hi nibble 0)
     * DLC 6, byte-identical to the VCM's next genuine frame by design -- that is
     * what makes a counter-validating GENE MCU reject the VCM's real command as
     * a duplicate and accept only our zero.
     */
    if (rx->data_length_code < 6)
    {
        s_tx_fail++;
        return;
    }

    twai_message_t tx = { 0 };
    tx.identifier = GEN_INHIBIT_VCM_ID;   /* 0x051 -- the real command ID */
    tx.data_length_code = 6;
    tx.data[0] = rx->data[0];
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
        s_tx_fail++;
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
        if (pos == GEN_INHIBIT_MMODE_VALUE)
        {
            s_disabled = true;
            s_disable_reason = "m_mode";
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

static void gen_inhibit_task(void *arg)
{
    static int64_t t_prev_051 = 0;
    twai_message_t rx;

    s_worker = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "worker started (prio %d)", GEN_INHIBIT_TASK_PRIO);

    while (1)
    {
        if (s_mode == GEN_INHIBIT_OFF)
        {
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
        else if (s_mode == GEN_INHIBIT_INHIBIT && !s_disabled)
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
                     "\"disabled\":%s,\"disable_reason\":\"%s\",\"soc_x100\":%lu,",
                     (int)s_mode, (unsigned long)s_offset_us, GEN_INHIBIT_PROBE_ID,
                     (unsigned long)s_tx_ok, (unsigned long)s_tx_fail,
                     (unsigned long)s_other_frames,
                     (unsigned long)s_ctr_steps_ok, (unsigned long)s_ctr_steps_bad,
                     can_is_enabled() ? "true" : "false",
                     s_we_enabled_bus ? "true" : "false",
                     (unsigned long)s_rx_errors,
                     s_disabled ? "true" : "false", s_disable_reason,
                     (unsigned long)s_soc_raw);

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
    gen_inhibit_reset_stats();
    xTaskCreate(gen_inhibit_task, "gen_inhibit", GEN_INHIBIT_STACK, NULL,
                GEN_INHIBIT_TASK_PRIO, NULL);

    if (GEN_INHIBIT_AUTOARM_MODE != GEN_INHIBIT_OFF)
    {
        ESP_LOGW(TAG, "auto-arm on boot: mode %d", (int)GEN_INHIBIT_AUTOARM_MODE);
        gen_inhibit_set_mode(GEN_INHIBIT_AUTOARM_MODE, GEN_INHIBIT_AUTOARM_OFFSET_US);
    }
}
