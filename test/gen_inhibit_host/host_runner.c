/*
 * host_runner -- drive gen_inhibit_core on a PC and print a deterministic
 * trace.
 *
 * This compiles THE SAME gen_inhibit_core.c the ESP32 runs. That makes it a
 * regression harness, not a differential one: it pins behaviour and catches
 * change, and it cannot catch a rule that is wrong here and wrong in the
 * golden too. Accepted deliberately -- see the header of gen_inhibit_core.h.
 *
 * WHAT IS EMULATED
 *
 * The worker loop in gen_inhibit.c, faithfully enough that frame ORDER and
 * the interaction between the periodic tick and frame arrival are reproduced,
 * because that is where all three bugs found by hand on 2026-09-19 lived:
 *
 *   OFF            gi_notify_off(), advance 50 ms, receive nothing. Frames
 *                  that arrive while OFF are DROPPED, exactly as they are on
 *                  the device, where the worker is not inside twai_receive().
 *   armed          gi_tick(), then a receive that blocks up to 200 ms. If the
 *                  next frame falls inside that window, time advances to it
 *                  and gi_on_frame() runs; otherwise time advances by the
 *                  full timeout and the next iteration ticks again.
 *
 * One frame is consumed per iteration so that a tick runs between any two
 * frames, as it does on the device.
 *
 * WHAT IS NOT EMULATED, AND THEREFORE NOT TESTED HERE
 *
 *   - the TWAI driver, can_enable()/can_disable(), and the listen-only dance
 *     for OBSERVE. Those live in the shim and are hardware-only.
 *   - real latency. Every transmit is instantaneous and succeeds unless a
 *     txfail window says otherwise, so the response histogram is all zeros.
 *     Timing is what the bench ESP-to-ESP test measures; it is not something
 *     a host replay can speak to.
 *   - preemption and the races between the worker and the HTTP handlers.
 *
 * INPUT (stdin), one directive per line, times in microseconds, ascending:
 *
 *   # comment
 *   cfg <field> <value>            before anything else; see cfg_set()
 *   mode <t> <mode> <offset_us>    0 off, 1 observe, 2 respond, 3 inhibit
 *   bus  <t> <en> <ours> <errvalid> <errcount>
 *   txfail <t_from> <t_to>         transmits in [from,to) fail
 *   f <t> <id_hex> <dlc> <hexbytes>
 *   end <t>                        stop; defaults to last frame + 1 s
 *
 * OUTPUT (stdout), one record per line, all values decimal unless noted.
 * Deterministic: no host clock, no addresses, no float.
 */
#include "gen_inhibit_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FRAMES  4000000
#define MAX_DIRS    4096

typedef struct { int64_t t; uint32_t id; uint8_t dlc; uint8_t data[8]; } rxf_t;

typedef enum { D_MODE, D_BUS, D_TXFAIL } dkind_t;
typedef struct { int64_t t; dkind_t k; int64_t a, b, c, d; } dir_t;

static rxf_t *g_f;
static long   g_nf, g_fi;
static dir_t  g_d[MAX_DIRS];
static int    g_nd, g_di;

static int64_t g_txfail_from = -1, g_txfail_to = -1;
static gi_bus_t g_bus;

/* ------------------------------------------------------------ trace out -- */

static int      g_last_valid;
static gi_state_t g_last;

static void hexdump(const uint8_t *d, int n, char *out)
{
    static const char *H = "0123456789ABCDEF";
    for (int i = 0; i < n; i++)
    {
        out[i * 2]     = H[(d[i] >> 4) & 0xF];
        out[i * 2 + 1] = H[d[i] & 0xF];
    }
    out[n * 2] = 0;
}

/*
 * STATE lines are emitted only when one of these fields changes. That keeps a
 * 100 Hz replay's trace readable and makes a diff point at the transition
 * that moved rather than at every frame after it.
 */
static void state_line(const gi_state_t *st, int64_t now)
{
    if (g_last_valid
        && g_last.mode == st->mode
        && g_last.inhibit_live == st->inhibit_live
        && g_last.arm_block == st->arm_block
        && g_last.abort_reason == st->abort_reason
        && g_last.disabled == st->disabled
        && g_last.disable_code == st->disable_code
        && g_last.shutdown_suppressed == st->shutdown_suppressed
        && g_last.fb_ever == st->fb_ever)
    {
        return;
    }
    printf("%lld STATE mode=%d live=%d block=%s abort=%s disabled=%d"
           " dcode=%s susp=%d fb_ever=%d\n",
           (long long)now, (int)st->mode, st->inhibit_live ? 1 : 0,
           gi_block_name(st->arm_block), gi_abort_name(st->abort_reason),
           st->disabled ? 1 : 0, gi_disable_name(st->disable_code),
           st->shutdown_suppressed ? 1 : 0, st->fb_ever ? 1 : 0);
    g_last = *st;
    g_last_valid = 1;
}

static void dump_events(const gi_events_t *ev)
{
    for (int i = 0; i < ev->n; i++)
    {
        const gi_event_t *e = &ev->e[i];
        printf("%lld EV %s a=%d b=%d c=%d\n", (long long)e->t_us,
               gi_event_name((gi_event_kind_t)e->kind), e->a, e->b, e->c);
    }
    if (ev->dropped)
    {
        printf("!! EVENT LIST OVERFLOW %u -- GI_EVENT_MAX too small\n",
               (unsigned)ev->dropped);
    }
}

/*
 * Transmit the core's emitted frames and feed the outcome back, the way
 * dispatch_emits() does on the device. due_us is honoured by moving the
 * simulated clock, not by spinning.
 */
static void dispatch(gi_state_t *st, const gi_emit_t *em, int64_t now)
{
    static const char *KIND[] = { "PROBE", "INHIBIT", "DIAG" };
    for (int i = 0; i < em->n; i++)
    {
        const gi_frame_t *f = &em->f[i];
        char hex[17];
        int64_t t_tx = f->have_due && f->due_us > now ? f->due_us : now;
        int ok = !(g_txfail_from >= 0 && t_tx >= g_txfail_from
                   && t_tx < g_txfail_to);

        hexdump(f->data, f->dlc, hex);
        printf("%lld TX %s id=%03X dlc=%u data=%s ok=%d\n",
               (long long)t_tx, KIND[f->kind], f->id, (unsigned)f->dlc,
               hex, ok);

        if (f->kind == GI_TX_DIAG)
        {
            continue;   /* best-effort on the device; not counted */
        }
        gi_events_t ev = { 0 };
        gi_on_tx_result(st, f, ok != 0, t_tx, &ev);
        dump_events(&ev);
    }
    if (em->dropped)
    {
        printf("!! EMIT LIST OVERFLOW %u -- GI_EMIT_MAX too small\n",
               (unsigned)em->dropped);
    }
}

/* ---------------------------------------------------------------- input -- */

static int cfg_set(gi_config_t *c, const char *k, long long v)
{
    if      (!strcmp(k, "soc_min_raw"))     c->soc_min_raw     = (uint32_t)v;
    else if (!strcmp(k, "soc_debounce"))    c->soc_debounce    = (uint32_t)v;
    else if (!strcmp(k, "start_abort_rpm")) c->start_abort_rpm = (int32_t)v;
    else if (!strcmp(k, "rpm_debounce_us")) c->rpm_debounce_us = v;
    else if (!strcmp(k, "fresh_us"))        c->fresh_us        = v;
    else if (!strcmp(k, "err_window_us"))   c->err_window_us   = v;
    else if (!strcmp(k, "err_min_trip"))    c->err_min_trip    = (uint32_t)v;
    else if (!strcmp(k, "diag_period_ms"))  c->diag_period_ms  = (uint32_t)v;
    else if (!strcmp(k, "max_rx_errors"))   c->max_rx_errors   = (uint32_t)v;
    else if (!strcmp(k, "fw_version"))      c->fw_version      = (uint16_t)v;
    else if (!strcmp(k, "git_hash"))        c->git_hash        = (uint32_t)v;
    else if (!strcmp(k, "autoarm"))         c->autoarm_configured = v != 0;
    else return 0;
    return 1;
}

static uint8_t hexnib(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0;
}

int main(void)
{
    gi_config_t cfg;
    gi_state_t  st;
    char line[512];
    int64_t t_end = -1;

    gi_config_defaults(&cfg);
    g_f = malloc(sizeof(rxf_t) * MAX_FRAMES);
    if (!g_f) { fprintf(stderr, "oom\n"); return 2; }

    /* Default bus: up, ours, error counter valid and quiet. */
    g_bus.can_enabled = true;
    g_bus.bus_ours = true;
    g_bus.err_valid = true;
    g_bus.bus_error_count = 0;

    while (fgets(line, sizeof(line), stdin))
    {
        char a[64], b[64];
        long long t, x, y, z, w;
        if (line[0] == '#' || line[0] == '\n') continue;

        if (sscanf(line, "cfg %63s %lld", a, &x) == 2)
        {
            if (!cfg_set(&cfg, a, x))
            {
                fprintf(stderr, "unknown cfg field '%s'\n", a);
                return 2;
            }
            continue;
        }
        if (sscanf(line, "mode %lld %lld %lld", &t, &x, &y) == 3)
        {
            if (g_nd >= MAX_DIRS) { fprintf(stderr, "too many dirs\n"); return 2; }
            g_d[g_nd++] = (dir_t){ t, D_MODE, x, y, 0, 0 };
            continue;
        }
        if (sscanf(line, "bus %lld %lld %lld %lld %lld", &t, &x, &y, &z, &w) == 5)
        {
            if (g_nd >= MAX_DIRS) { fprintf(stderr, "too many dirs\n"); return 2; }
            g_d[g_nd++] = (dir_t){ t, D_BUS, x, y, z, w };
            continue;
        }
        if (sscanf(line, "txfail %lld %lld", &t, &x) == 2)
        {
            if (g_nd >= MAX_DIRS) { fprintf(stderr, "too many dirs\n"); return 2; }
            g_d[g_nd++] = (dir_t){ t, D_TXFAIL, t, x, 0, 0 };
            continue;
        }
        if (sscanf(line, "end %lld", &t) == 1) { t_end = t; continue; }

        if (sscanf(line, "f %lld %63s %lld %63s", &t, a, &x, b) == 4)
        {
            if (g_nf >= MAX_FRAMES) { fprintf(stderr, "too many frames\n"); return 2; }
            rxf_t *f = &g_f[g_nf++];
            memset(f, 0, sizeof(*f));
            f->t = t;
            f->id = (uint32_t)strtoul(a, NULL, 16);
            f->dlc = (uint8_t)x;
            for (int i = 0; i < f->dlc && b[i * 2] && b[i * 2 + 1]; i++)
            {
                f->data[i] = (uint8_t)((hexnib(b[i * 2]) << 4)
                                       | hexnib(b[i * 2 + 1]));
            }
            continue;
        }
        fprintf(stderr, "unparsed: %s", line);
        return 2;
    }

    gi_init(&st, &cfg);

    int64_t now = 0;
    if (g_nf > 0) now = g_f[0].t;
    if (g_nd > 0 && g_d[0].t < now) now = g_d[0].t;
    if (t_end < 0) t_end = (g_nf > 0 ? g_f[g_nf - 1].t : now) + 1000000;

    printf("# host_runner: %ld frames, %d directives, t=[%lld,%lld]\n",
           g_nf, g_nd, (long long)now, (long long)t_end);
    state_line(&st, now);

    for (;;)
    {
        /* Directives due now. */
        while (g_di < g_nd && g_d[g_di].t <= now)
        {
            const dir_t *d = &g_d[g_di++];
            gi_events_t ev = { 0 };
            switch (d->k)
            {
            case D_MODE:
                gi_set_mode(&st, (gi_mode_t)d->a, (uint32_t)d->b, now, &ev);
                break;
            case D_BUS:
                g_bus.can_enabled = d->a != 0;
                g_bus.bus_ours = d->b != 0;
                g_bus.err_valid = d->c != 0;
                g_bus.bus_error_count = (uint32_t)d->d;
                break;
            case D_TXFAIL:
                g_txfail_from = d->a;
                g_txfail_to = d->b;
                break;
            }
            dump_events(&ev);
            state_line(&st, now);
        }

        if (now >= t_end && g_fi >= g_nf) break;

        if (st.mode == GI_OFF)
        {
            gi_notify_off(&st);
            state_line(&st, now);
            /*
             * Frames arriving while OFF are dropped: the device's worker is
             * not inside twai_receive() on this path.
             */
            int64_t nxt = now + 50000;
            while (g_fi < g_nf && g_f[g_fi].t < nxt) g_fi++;
            now = nxt;
            if (now > t_end && g_fi >= g_nf) break;
            continue;
        }

        {
            gi_emit_t em = { 0 };
            gi_events_t ev = { 0 };
            gi_tick(&st, now, &g_bus, &em, &ev);
            dispatch(&st, &em, now);
            dump_events(&ev);
            state_line(&st, now);
        }

        int64_t deadline = now + 200000;   /* GEN_INHIBIT_RX_TIMEOUT_MS */
        int64_t nxt = deadline;
        if (g_fi < g_nf && g_f[g_fi].t < nxt) nxt = g_f[g_fi].t;
        if (g_di < g_nd && g_d[g_di].t < nxt) nxt = g_d[g_di].t;
        if (nxt < now) nxt = now;
        now = nxt;

        if (g_fi < g_nf && g_f[g_fi].t <= now)
        {
            const rxf_t *f = &g_f[g_fi++];
            gi_emit_t em = { 0 };
            gi_events_t ev = { 0 };
            gi_on_rx_ok(&st);
            gi_on_frame(&st, f->id, f->dlc, f->data, now, &em, &ev);
            dispatch(&st, &em, now);
            dump_events(&ev);
            state_line(&st, now);
        }
    }

    printf("%lld FINAL mode=%d live=%d tx_ok=%u tx_fail=%u other=%u"
           " ctr_ok=%u ctr_bad=%u rx_gap_n=%u resp_n=%u disabled=%d"
           " dcode=%s block=%s abort=%s soc=%u shift=%u\n",
           (long long)now, (int)st.mode, st.inhibit_live ? 1 : 0,
           st.tx_ok, st.tx_fail, st.other_frames,
           st.ctr_steps_ok, st.ctr_steps_bad,
           st.rx_gap.count, st.response.count, st.disabled ? 1 : 0,
           gi_disable_name(st.disable_code), gi_block_name(st.arm_block),
           gi_abort_name(st.abort_reason), st.soc_raw,
           (unsigned)st.last_shift_pos);

    free(g_f);
    return 0;
}
