/*
 * gen_inhibit_core -- the decision logic. See gen_inhibit_core.h for the
 * rules this file obeys and for why it exists at all.
 *
 * Nothing in here may include a platform header, read a clock, allocate, or
 * log. If a change needs one of those, it belongs in gen_inhibit.c.
 *
 * This file is a behaviour-preserving extraction from gen_inhibit.c as it
 * stood on 2026-09-19. Where a comment explains WHY a rule is the way it is,
 * it has been carried across verbatim in substance -- those justifications
 * are the record of what was measured on the truck, and losing them in a
 * refactor would be the expensive part.
 */
#include "gen_inhibit_core.h"

#include <string.h>

/* ---------------------------------------------------------------- config -- */

void gi_config_defaults(gi_config_t *c)
{
    memset(c, 0, sizeof(*c));

    /*
     * 21 %, deliberately a full percentage point above the truck's own 20.0 %
     * trigger, so we get out of the way BEFORE the VCM asks for the engine
     * rather than at the same instant. The cost is giving up inhibiting in
     * the 20-21 % band; the benefit is that there is no window in which we
     * are blocking a start the pack genuinely needs. Spec section 6.2:
     * isolating entries into charge-sustain across the whole corpus leaves
     * two events, firing at 20.02 % and 20.23 %, so the truck defends a
     * 20.0 % floor.
     */
    c->soc_min_raw = 21 * 100;

    /*
     * 0x411 reads 0 for the first samples at startup before it is valid. A
     * raw 0 is treated as "not yet valid", never as 0 %, and the
     * sub-threshold condition must persist this many valid samples before it
     * latches -- so a transient 0->real-value at boot cannot false-trip.
     */
    c->soc_debounce = 5;

    /*
     * 300 rpm sustained for 0.3 s, from the laptop tool's --start-abort-rpm.
     * The debounce is not optional: a momentary crank blip is exactly what a
     * WORKING inhibit produces, and aborting on it would end the run the
     * inhibit just won.
     */
    c->start_abort_rpm = 300;
    c->rpm_debounce_us = 300000;

    /*
     * Freshness, not last-known values: a silent bus otherwise answers every
     * question you ask it. 0.5 s matches the reference tool's gene_alive().
     */
    c->fresh_us = 500000;

    /*
     * Error frames are rate-based, never first-strike. Aborting on the first
     * one ended three consecutive armed runs. The running-generator regime
     * produces 0.07-0.21/s and bursts to 4 in any 10 s; a background measured
     * engine-off reads ~0/s and is useless for calibration. 10 in 10 s keeps
     * 2.5x headroom over the worst observed burst, while a genuine same-ID
     * collision at 100 Hz would blow past it inside a second.
     */
    c->err_window_us = 10000000LL;
    c->err_min_trip  = 10;

    c->diag_period_ms = 300;

    /*
     * A run of hard receive errors means the driver is gone underneath us --
     * for example another subsystem called can_disable(). Disarm rather than
     * keep retrying, so the device parks itself in a safe, answerable state
     * instead of needing a power cycle.
     */
    c->max_rx_errors = 20;

    c->fw_version = 1;
    c->git_hash   = 0;
    c->autoarm_configured = false;
}

/* ----------------------------------------------------------- reason names -- */

/*
 * LOAD-BEARING STRINGS. These are what /gen_inhibit reports as "arm_block"
 * and "abort_reason", so anything that greps them is coupled to this table.
 * See the warning in the header before changing one.
 */
const char *gi_block_name(gi_block_t b)
{
    switch (b)
    {
    case GI_BLOCK_NONE:                return "";
    case GI_BLOCK_NOT_ARMED:           return "not armed";
    case GI_BLOCK_GATE_NOT_EVALUATED:  return "gate not yet evaluated";
    case GI_BLOCK_NO_FRESH_CMD:        return "no fresh 0x051";
    case GI_BLOCK_VCM_FAULT:           return "VCM fault active";
    case GI_BLOCK_GEN_RUNNING:         return "generator running";
    case GI_BLOCK_VCM_REQ_ENGINE:      return "VCM requesting engine";
    case GI_BLOCK_VCM_TORQUE:          return "VCM commanding torque";
    case GI_BLOCK_SHUTDOWN_CMD:        return "VCM commanding 0x10";
    case GI_BLOCK_DISABLED:            return "latched disable";
    }
    return "?";
}

const char *gi_abort_name(gi_abort_t a)
{
    switch (a)
    {
    case GI_ABORT_NONE:            return "";
    case GI_ABORT_TX_FAILED:       return "transmit failed";
    case GI_ABORT_VCM_FAULT:       return "0x617 B7 = 0xCA (VCM fault active)";
    case GI_ABORT_BUS_LOST:        return "bus lost -- no 0x051 (latched; CAN link down)";
    case GI_ABORT_INVERTER_LOST:   return "0x471 stopped while 0x051 still live -- inverter lost";
    case GI_ABORT_ENGINE_TURNING:  return "engine turning while armed -- the inhibit did not hold";
    case GI_ABORT_ERROR_RATE:      return "error-frame rate exceeded";
    }
    return "?";
}

const char *gi_disable_name(gi_disable_t d)
{
    switch (d)
    {
    case GI_DISABLE_NONE:     return "";
    case GI_DISABLE_M_MODE:   return "m_mode";
    case GI_DISABLE_LOW_SOC:  return "low_soc";
    }
    return "?";
}

const char *gi_event_name(gi_event_kind_t k)
{
    switch (k)
    {
    case GI_EV_NONE:                return "none";
    case GI_EV_DISABLED:            return "DISABLED";
    case GI_EV_SHUTDOWN_SUPPRESS:   return "SHUTDOWN_SUPPRESS";
    case GI_EV_INHIBIT_LIVE:        return "INHIBIT_LIVE";
    case GI_EV_ABORT:               return "ABORT";
    case GI_EV_TX_FAIL:             return "TX_FAIL";
    case GI_EV_RX_ERROR_DISARM:     return "RX_ERROR_DISARM";
    case GI_EV_MODE:                return "MODE";
    }
    return "?";
}

/* -------------------------------------------------------------- plumbing -- */

static void emit(gi_emit_t *out, const gi_frame_t *f)
{
    if (out == NULL)
    {
        return;
    }
    if (out->n >= GI_EMIT_MAX)
    {
        /* A bug in this file, not a condition to handle. See the header. */
        out->dropped++;
        return;
    }
    out->f[out->n++] = *f;
}

static void ev_add(gi_events_t *ev, int64_t t, gi_event_kind_t k,
                   int32_t a, int32_t b, int32_t c)
{
    if (ev == NULL)
    {
        return;
    }
    if (ev->n >= GI_EVENT_MAX)
    {
        ev->dropped++;
        return;
    }
    gi_event_t *e = &ev->e[ev->n++];
    e->t_us = t;
    e->kind = (uint8_t)k;
    e->a = a;
    e->b = b;
    e->c = c;
}

/* ------------------------------------------------------------ histograms -- */

const uint32_t gi_bucket_us[GI_NBUCKETS] = {
    50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, 0xFFFFFFFFu
};

void gi_hist_reset(gi_hist_t *h)
{
    memset(h, 0, sizeof(*h));
    h->min_us = 0xFFFFFFFFu;
}

void gi_hist_add(gi_hist_t *h, uint32_t us)
{
    h->count++;
    h->sum_us += us;
    if (us < h->min_us) h->min_us = us;
    if (us > h->max_us) h->max_us = us;
    for (int i = 0; i < GI_NBUCKETS; i++)
    {
        if (us < gi_bucket_us[i])
        {
            h->buckets[i]++;
            return;
        }
    }
}

/* ------------------------------------------------------------ extractors -- */

int32_t gi_le16c(const uint8_t *d, int32_t zero)
{
    return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8)) - zero;
}

uint32_t gi_soc_raw(const uint8_t *d)
{
    /* BMS_SoC_HiRes: 14-bit big-endian at bit 7, scale 0.01 %. */
    return ((uint32_t)d[0] << 6) | (uint32_t)(d[1] >> 2);
}

uint8_t gi_shift_pos(const uint8_t *d)
{
    return (uint8_t)((d[6] >> 4) & 0x07);
}

uint8_t gi_ctr(const uint8_t *d)
{
    return (uint8_t)(d[5] & 0x0F);
}

bool gi_fresh(const gi_state_t *st, bool have, int64_t stamp, int64_t now)
{
    /*
     * Plain signed comparison, deliberately. See the time note in the header:
     * this is int64 microseconds and does not wrap, so the unsigned-
     * subtraction idiom the interposer core needs would be wrong here.
     */
    return have && (now - stamp) < st->cfg.fresh_us;
}

/* ------------------------------------------------------------ lifecycle -- */

void gi_reset_stats(gi_state_t *st)
{
    gi_hist_reset(&st->rx_gap);
    gi_hist_reset(&st->response);
    st->tx_ok = st->tx_fail = st->other_frames = 0;
    st->ctr_steps_ok = st->ctr_steps_bad = 0;
    st->have_last_ctr = false;
    st->rx_errors = 0;

    /*
     * Live bus-derived state, not a statistic, but it must not survive an arm
     * cycle: disarming mid-0x10 and re-arming would otherwise start
     * suppressed on a stale reading until the next 0x051. It is recomputed
     * from the first frame either way; this just makes the starting point
     * honest.
     */
    st->shutdown_suppressed = false;

    /*
     * Spec 7: a new arm starts from a clean gate. Signal freshness is
     * deliberately NOT cleared -- it is a property of the bus, not of this
     * run, and dropping it would make every arm wait a fresh round before the
     * interlocks could pass.
     */
    st->inhibit_live = false;
    st->arm_block    = GI_BLOCK_GATE_NOT_EVALUATED;
    st->abort_reason = GI_ABORT_NONE;
    st->rpm_over = false;
    st->rpm_over_since = 0;
    st->have_err_window = false;
    st->err_window = 0;
    st->fb_ever = false;

    /*
     * Not in the pre-refactor reset, but it was reset in the worker's OFF
     * branch for exactly the same reason and with the same comment: a stale
     * previous-0x051 would make the first gap after the next arm the whole
     * idle interval, silently poisoning every run after the first. Doing it
     * here as well is harmless -- the value is only read when have_prev_051
     * is set -- and makes the histogram reset complete in one place.
     */
    st->have_prev_051 = false;
    st->prev_051 = 0;
}

void gi_init(gi_state_t *st, const gi_config_t *cfg)
{
    memset(st, 0, sizeof(*st));
    if (cfg != NULL)
    {
        st->cfg = *cfg;
    }
    else
    {
        gi_config_defaults(&st->cfg);
    }

    st->mode = GI_OFF;
    st->offset_us = 500;
    st->last_shift_pos = 0xFF;      /* never seen */
    st->vcm_fault = 0xC8;           /* no fault */
    st->disable_code = GI_DISABLE_NONE;
    gi_reset_stats(st);
    st->arm_block = GI_BLOCK_NOT_ARMED;
}

void gi_set_mode(gi_state_t *st, gi_mode_t mode, uint32_t offset_us,
                 int64_t now, gi_events_t *ev)
{
    st->offset_us = offset_us;
    /*
     * Reset only when arming. Clearing on disarm too meant a client that did
     * the obvious thing -- stop, then read the results -- got zeros back,
     * which is indistinguishable from "the device received nothing".
     * Statistics now survive the disarm and live until the next arm.
     */
    if (mode != GI_OFF)
    {
        gi_reset_stats(st);
    }
    st->mode = mode;
    ev_add(ev, now, GI_EV_MODE, (int32_t)mode, (int32_t)offset_us, 0);
}

void gi_notify_off(gi_state_t *st)
{
    /*
     * Clear the live flag on EVERY route into OFF, not just on the abort
     * path. gi_reset_stats() only runs when arming, so an ordinary disarm
     * would otherwise leave inhibit_live set: harmless for transmission,
     * since the dispatch also tests the mode, but it would make diag_flags
     * bit5 and the JSON claim a live inhibit on a disarmed device -- exactly
     * the kind of reassuring-but-wrong reading these flags exist to prevent.
     */
    if (st->inhibit_live)
    {
        st->inhibit_live = false;
        st->arm_block = GI_BLOCK_NOT_ARMED;
    }
    /*
     * Unconditionally, not just on release: a stale previous-0x051 would make
     * the first gap after the next arm the whole idle interval, silently
     * poisoning every run after the first.
     */
    st->have_prev_051 = false;
    st->prev_051 = 0;
}

/* ------------------------------------------------------------- interlocks -- */

static void inhibit_abort(gi_state_t *st, gi_abort_t why, int64_t now,
                          gi_events_t *ev)
{
    st->abort_reason = why;
    st->inhibit_live = false;
    st->mode = GI_OFF;
    ev_add(ev, now, GI_EV_ABORT, (int32_t)why, 0, 0);
}

/*
 * Spec 7: may the inhibit go live right now? Every check is against FRESHNESS
 * as well as value -- a silent bus otherwise answers every question you ask
 * it, and answers them all reassuringly.
 */
static bool arm_gate_ok(gi_state_t *st, int64_t now)
{
    if (!gi_fresh(st, st->have_cmd, st->seen_cmd, now))
    {
        st->arm_block = GI_BLOCK_NO_FRESH_CMD;
        return false;
    }
    /*
     * 0x471 and 0x054 are deliberately NOT required here, and that is a
     * correction (2026-09-19) rather than an omission.
     *
     * Measured across bus wake events in the corpus: after the bus comes
     * alive, 0x051 appears within ~0.17 s, but the GENE family (0x471, 0x054)
     * appears +28 s later, or not at all within 60 s -- the inverter powers
     * up long after the VCM, and in sessions where the generator is never
     * used it never appears. Requiring them to arm would mean the inhibit
     * went live ~28 s after key-on at best and never at worst, while the
     * evap-driven start it exists to prevent follows key-on almost
     * immediately.
     *
     * It is also self-defeating: requiring fresh 0x471 means arming only once
     * the inverter is powered, which is the neighbourhood of the state spec 7
     * explicitly refuses to arm into. Their absence is positive evidence the
     * generator is NOT running.
     *
     * Spec 7 lists 0x471 only as a DISARM condition -- "disarm if the
     * inverter goes quiet (0x471 stops)" -- and "stops" presupposes it was
     * going. The runtime trip below honours that by firing only once it has
     * been seen.
     */
    if (st->vcm_fault == GI_FAULT_ACTIVE)
    {
        st->arm_block = GI_BLOCK_VCM_FAULT;
        return false;
    }
    /*
     * Refuse while the generator is already running or the VCM is asking for
     * torque. Taking over a loaded generator and commanding zero sheds the
     * engine's whole load in one frame -- a load dump on a running engine,
     * and the wrong way to find out whether the mechanism works. The only
     * transition this arms into is "engine off, VCM tries to start it, we
     * stop it".
     *
     * Only meaningful if 0x054 is actually arriving. A stale reading cannot
     * say the generator is running -- but nor can it say it is stopped, so
     * this leans on the GENE family's silence being evidence of an unpowered
     * inverter rather than trusting a last-known value.
     */
    if (gi_fresh(st, st->have_rpm, st->seen_rpm, now)
        && st->gene_rpm >= st->cfg.start_abort_rpm)
    {
        st->arm_block = GI_BLOCK_GEN_RUNNING;
        return false;
    }
    if (st->vcm_rpm_ref >= 0)
    {
        st->arm_block = GI_BLOCK_VCM_REQ_ENGINE;
        return false;
    }
    if (st->vcm_torque != 0)
    {
        st->arm_block = GI_BLOCK_VCM_TORQUE;
        return false;
    }
    if (st->shutdown_suppressed)
    {
        st->arm_block = GI_BLOCK_SHUTDOWN_CMD;
        return false;
    }
    st->arm_block = GI_BLOCK_NONE;
    return true;
}

/*
 * Spec 7: the trips that end a live run. Called every loop, not only on an
 * 0x051, so a bus that goes quiet is caught by the receive timeout.
 */
static void interlock_runtime(gi_state_t *st, int64_t now, const gi_bus_t *bus,
                              gi_events_t *ev)
{
    if (st->vcm_fault == GI_FAULT_ACTIVE)
    {
        inhibit_abort(st, GI_ABORT_VCM_FAULT, now, ev);
        return;
    }

    /*
     * THE BUS ITSELF HAS GONE. 0x051 runs at ~100 Hz whenever the link is up,
     * so its absence is the anchor test for "we are no longer connected to
     * the powertrain bus" -- a pulled connector, a dropped transceiver,
     * key-off. Framed as link loss deliberately, rather than as individual
     * signals going offline: the realistic failure is the CAN connection
     * dropping entirely, in which case every signal stops together and 0x051
     * is the one to notice it by.
     *
     * THIS LATCHES. Decided 2026-09-19. The tempting alternative is to stand
     * down and re-arm when the bus returns, on the grounds that link loss is
     * an absence rather than a fault and a momentary glitch should not cost a
     * drive. That is rejected: if the link is INTERMITTENT, resuming is the
     * wrong response. A bus that drops and returns is an unreliable
     * environment, and this device does not merely observe it -- it steals
     * the VCM's rolling counter and transmits a real 0x051 onto a live
     * powertrain bus. Coming back automatically would mean doing that
     * repeatedly across a link we already have evidence is unsound, and each
     * resume would re-enter through a gate whose freshness checks a flapping
     * bus can satisfy. Latching turns an intermittent connection into one
     * clean stand-down instead of an oscillation.
     *
     * Latched like the section 6 releases: cleared only by a reboot, which
     * the relay dropping at truck sleep provides. A consequence worth knowing
     * is that a key-off/key-on quick enough that the relay never opens leaves
     * the inhibit latched off for that second drive -- which is the intended
     * reading of "do not resume", not an oversight.
     *
     * THIS REPLACES THE DEAD-MAN TIMER, which is removed (spec 7,
     * 2026-09-19). Every hazard the timer was still covering reduces to "the
     * signals that would release us stopped arriving", and losing the VCU's
     * 0x051 and the BCM's 0x411 together has no plausible cause on a healthy
     * link -- it IS link loss, which this catches directly and by evidence
     * rather than by elapsed time.
     */
    if (!gi_fresh(st, st->have_cmd, st->seen_cmd, now))
    {
        inhibit_abort(st, GI_ABORT_BUS_LOST, now, ev);
        return;
    }

    /*
     * "Disarm if the inverter goes QUIET (0x471 STOPS)" -- so this can only
     * fire once 0x471 has actually been heard. Without fb_ever the trip would
     * fire the instant we went live on a bus whose inverter has not powered
     * up yet, which measurement says is the normal case for the first ~28 s
     * after key-on and is permanent in sessions where the generator is never
     * used. Never-seen is not the same as stopped.
     *
     * 0x051 is known fresh by this point, so this is unambiguously the
     * inverter specifically dropping off a live bus -- the alarming case, and
     * the distinction the laptop tool could not make. Whole-bus loss was
     * handled above and is not a fault.
     */
    if (st->fb_ever && !gi_fresh(st, st->have_fb, st->seen_fb, now))
    {
        inhibit_abort(st, GI_ABORT_INVERTER_LOST, now, ev);
        return;
    }

    /*
     * Note this reads gene_rpm WITHOUT a freshness test, where the arm gate
     * above tests freshness. That asymmetry is pre-refactor behaviour and is
     * preserved deliberately rather than tidied: once live, the last known
     * "the engine is turning" is the conservative reading, and whole-bus loss
     * has already been handled above. Flagged in the verification table as
     * something to confirm on the bench rather than assume.
     */
    if (st->gene_rpm >= st->cfg.start_abort_rpm)
    {
        if (!st->rpm_over)
        {
            st->rpm_over = true;
            st->rpm_over_since = now;
        }
        else if (now - st->rpm_over_since > st->cfg.rpm_debounce_us)
        {
            inhibit_abort(st, GI_ABORT_ENGINE_TURNING, now, ev);
            return;
        }
    }
    else
    {
        st->rpm_over = false;
        st->rpm_over_since = 0;
    }

    /*
     * Error frames, rate-based over a sliding window. Note this counts the
     * controller's own bus_error_count rather than anything payload-derived,
     * so it is distinct from the failed-transmit trip, which is unambiguously
     * about a frame of ours.
     */
    if (bus != NULL && bus->err_valid)
    {
        if (!st->have_err_window
            || (now - st->err_window) > st->cfg.err_window_us)
        {
            st->have_err_window = true;
            st->err_window = now;
            st->err_base = bus->bus_error_count;
        }
        else if ((uint32_t)(bus->bus_error_count - st->err_base)
                 >= st->cfg.err_min_trip)
        {
            inhibit_abort(st, GI_ABORT_ERROR_RATE, now, ev);
            return;
        }
    }
}

/* ------------------------------------------------------------------ diag -- */

static void build_diag(const gi_state_t *st, const gi_bus_t *bus,
                       uint8_t which, gi_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->dlc = 8;
    f->kind = GI_TX_DIAG;

    if (which == 0)     /* STATUS -> 0x7F1 */
    {
        uint8_t flags = (uint8_t)
              ((st->disabled                 ? 0x01 : 0)
             | ((bus && bus->can_enabled)    ? 0x02 : 0)
             | ((bus && bus->bus_ours)       ? 0x04 : 0)
             | (st->cfg.autoarm_configured   ? 0x08 : 0)
             /*
              * bit4: live, non-latching. Spec 6.4 puts this in the flags
              * rather than disable_reason precisely because it comes and goes
              * -- a reason code that appeared and vanished as the truck
              * parked and woke could not be told apart from a past one when
              * reading a log.
              */
             | (st->shutdown_suppressed      ? 0x10 : 0)
             /*
              * bit5: spec 7 interlocks satisfied and transmitting. Distinct
              * from mode 3, which is only what was asked for -- the two
              * differ for the whole time the gate is waiting on the bus, and
              * that gap is the interesting thing to see in a log.
              */
             | (st->inhibit_live             ? 0x20 : 0));

        f->id = GI_DIAG_ID_STATUS;
        f->data[0] = GI_DIAG_SCHEMA_VER;
        f->data[1] = (uint8_t)st->cfg.fw_version;
        f->data[2] = (uint8_t)st->mode;
        f->data[3] = flags;
        f->data[4] = (uint8_t)st->disable_code;
        f->data[5] = st->last_shift_pos;
        f->data[6] = (uint8_t)st->soc_raw;          /* soc_x100, LE */
        f->data[7] = (uint8_t)(st->soc_raw >> 8);
    }
    else if (which == 1)    /* COUNTERS -> 0x7F2 */
    {
        uint32_t v = st->tx_ok;
        f->id = GI_DIAG_ID_COUNTERS;
        f->data[0] = (uint8_t)v;
        f->data[1] = (uint8_t)(v >> 8);
        f->data[2] = (uint8_t)(v >> 16);
        f->data[3] = (uint8_t)(v >> 24);
        f->data[4] = st->tx_fail       > 255 ? 255 : (uint8_t)st->tx_fail;
        f->data[5] = st->ctr_steps_bad > 255 ? 255 : (uint8_t)st->ctr_steps_bad;
        f->data[6] = st->rx_errors     > 255 ? 255 : (uint8_t)st->rx_errors;
    }
    else                    /* BUILD -> 0x7F3 */
    {
        uint32_t h = st->cfg.git_hash;
        f->id = GI_DIAG_ID_BUILD;
        f->data[0] = (uint8_t)h;
        f->data[1] = (uint8_t)(h >> 8);
        f->data[2] = (uint8_t)(h >> 16);
        f->data[3] = (uint8_t)(h >> 24);
        f->data[4] = (uint8_t)(st->cfg.fw_version);
        f->data[5] = (uint8_t)(st->cfg.fw_version >> 8);
        f->data[6] = GI_DIAG_SCHEMA_VER;
    }
}

/* ------------------------------------------------------------------ tick -- */

void gi_tick(gi_state_t *st, int64_t now, const gi_bus_t *bus,
             gi_emit_t *out, gi_events_t *ev)
{
    if (st->mode == GI_OFF)
    {
        return;
    }

    /*
     * Self-telemetry heartbeat, round-robin across the three diag IDs. Runs
     * before the blocking receive so it still fires on a quiet bus. Armed
     * only.
     */
    if (!st->have_last_diag
        || (now - st->last_diag) >= (int64_t)st->cfg.diag_period_ms * 1000)
    {
        gi_frame_t f;
        st->have_last_diag = true;
        st->last_diag = now;
        build_diag(st, bus, st->diag_page, &f);
        emit(out, &f);
        st->diag_page = (uint8_t)((st->diag_page + 1) % 3);
    }

    /*
     * LOAD-BEARING INVARIANT: interlock_runtime() runs ONLY once the inhibit
     * is live, never on the path to going live. The bus-loss trip inside it
     * latches, and before key-on there is legitimately no 0x051 at all
     * (measured: the PT bus is silent until the key turns, and the GENE
     * family for ~28 s after that). Running the trips on a not-yet-live
     * inhibit would therefore latch the device off during the ordinary
     * pre-key-on wait -- on an auto-arm build, before it had ever inhibited
     * anything. Absence of the bus BEFORE going live is "not ready yet",
     * handled by arm_gate_ok(); absence AFTER going live is "the link we were
     * using has dropped". Do not merge these paths.
     */
    if (st->mode == GI_INHIBIT && st->disabled)
    {
        /*
         * Latched off. latch_disable() already cleared the live flag at the
         * instant of the latch; this covers the other route to the same
         * state -- re-arming while a release is still latched, where
         * gi_reset_stats() would otherwise leave arm_block reading
         * "gate not yet evaluated" and hide the reason the gate will never
         * run. Belt and braces on the flag too, since it is load-bearing.
         */
        st->inhibit_live = false;
        st->arm_block = GI_BLOCK_DISABLED;
    }
    else if (st->mode == GI_INHIBIT)
    {
        if (st->inhibit_live)
        {
            interlock_runtime(st, now, bus, ev);
        }
        else if (arm_gate_ok(st, now))
        {
            st->inhibit_live = true;
            st->rpm_over = false;
            st->rpm_over_since = 0;
            st->have_err_window = false;
            st->err_window = 0;
            st->abort_reason = GI_ABORT_NONE;
            ev_add(ev, now, GI_EV_INHIBIT_LIVE, 0, 0, 0);
        }
    }
}

/* ----------------------------------------------------------------- frames -- */

/*
 * Conditions that DISABLE the inhibit, latched until reboot. Both mean the
 * engine/generator is legitimately wanted, so we stop transmitting and stay
 * stopped (power-cycle -- the relay drops at truck sleep -- to re-enable).
 *
 *   M mode  -- driver manually commanding the engine (0x639 shift_lever_pos
 *              == 4, "Manual_generator_mode", confirmed in the powertrain
 *              DBC).
 *   Low SoC -- pack needs the generator to charge (0x411 BMS_SoC_HiRes).
 */
/*
 * Latch a section 6 release.
 *
 * Spec 6.4 (2026-09-19): a latched release is a STAND-DOWN, so the section 7
 * live flag clears with it. This is not cosmetic. `inhibit_live` is one of the
 * terms that authorises a transmit in gi_on_frame(), so leaving it set means a
 * control flag reads "authorised" while transmission is forbidden, and the
 * only thing preventing a transmit is the `!disabled` term sitting beside it
 * in the same && chain. That is a one-term margin that depends on the order of
 * a boolean expression, and it is exactly the kind of thing a later edit
 * breaks silently.
 *
 * Cleared here, at the instant of the latch, rather than on the next tick, so
 * that the dispatch later in this same gi_on_frame() call already sees it
 * false and does not rely on `!disabled` at all.
 *
 * Found by the host suite. The pre-refactor code left the flag frozen true for
 * the rest of the run, because the whole interlock block is guarded by
 * `mode == INHIBIT && !disabled` and the worker never reaches its OFF branch
 * while the mode is still INHIBIT.
 */
static void latch_disable(gi_state_t *st, gi_disable_t why, int32_t detail,
                          int64_t now, gi_events_t *ev)
{
    st->disabled = true;
    st->disable_code = why;
    st->inhibit_live = false;
    st->arm_block = GI_BLOCK_DISABLED;
    ev_add(ev, now, GI_EV_DISABLED, (int32_t)why, detail, 0);
}

static void disable_monitor(gi_state_t *st, uint32_t id, uint8_t dlc,
                            const uint8_t *data, int64_t now, gi_events_t *ev)
{
    if (st->disabled)
    {
        return;     /* latched until reboot; nothing more to evaluate */
    }

    if (id == GI_MMODE_ID && dlc >= 7)
    {
        uint8_t pos = gi_shift_pos(data);
        st->last_shift_pos = pos;
        if (pos == GI_MMODE_VALUE)
        {
            latch_disable(st, GI_DISABLE_M_MODE, 0, now, ev);
        }
    }
    else if (id == GI_SOC_ID && dlc >= 2)
    {
        uint32_t raw = gi_soc_raw(data);
        if (raw == 0)
        {
            /* Startup sentinel, not 0 % -- treat as not-yet-valid. */
            st->soc_low_count = 0;
        }
        else
        {
            st->soc_raw = raw;
            if (raw < st->cfg.soc_min_raw)
            {
                if (++st->soc_low_count >= st->cfg.soc_debounce)
                {
                    latch_disable(st, GI_DISABLE_LOW_SOC, (int32_t)raw,
                                  now, ev);
                }
            }
            else
            {
                st->soc_low_count = 0;
            }
        }
    }
}

/*
 * Record every interlock signal's value AND the time it was last seen. Runs
 * on every frame whatever the mode, so the gate has history the instant a
 * mode change asks for it, rather than having to wait for a fresh round after
 * the request.
 */
static void interlock_monitor(gi_state_t *st, uint32_t id, uint8_t dlc,
                              const uint8_t *data, int64_t now)
{
    switch (id)
    {
    case GI_VCM_ID:
        st->have_cmd = true;
        st->seen_cmd = now;
        if (dlc >= 5)
        {
            st->vcm_torque  = gi_le16c(&data[1], GI_CMD_ZERO);
            st->vcm_rpm_ref = gi_le16c(&data[3], GI_CMD_ZERO);
        }
        break;
    case GI_FB_ID:
        /* Liveness only. The value is not used; that it arrives is the test. */
        st->have_fb = true;
        st->seen_fb = now;
        st->fb_ever = true;
        break;
    case GI_RPM_ID:
        st->have_rpm = true;
        st->seen_rpm = now;
        if (dlc >= 2)
        {
            st->gene_rpm = gi_le16c(&data[0], GI_RPM_ZERO);
        }
        break;
    case GI_FAULT_ID:
        if (dlc >= 8)
        {
            st->vcm_fault = data[7];
        }
        break;
    default:
        break;
    }
}

static void build_probe(const gi_state_t *st, int64_t t_rx, gi_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id = GI_PROBE_ID;
    f->dlc = 8;
    f->kind = GI_TX_PROBE;

    /* Echo the RX timestamp so a capture can be aligned without guessing. */
    uint32_t lo = (uint32_t)t_rx;
    f->data[0] = (uint8_t)(lo);
    f->data[1] = (uint8_t)(lo >> 8);
    f->data[2] = (uint8_t)(lo >> 16);
    f->data[3] = (uint8_t)(lo >> 24);
    f->data[4] = (uint8_t)(st->offset_us);
    f->data[5] = (uint8_t)(st->offset_us >> 8);
    f->data[6] = st->last_ctr;
    f->data[7] = 0x5A;

    f->have_due = true;
    f->due_us   = t_rx + (int64_t)st->offset_us;
    f->have_t_rx = true;
    f->t_rx = t_rx;
}

/*
 * The truck-validated inhibit frame (DBC + inhibit.py + 18k decoded frames).
 * Rebuilt from the VCM frame we just received:
 *   B0     HELD     0x08, the engine-off state (spec 4.1; was mirrored)
 *   B1,B2  00 80    gen_torque_cmd = 0 (raw 0x8000, 16-bit LE, offset -32768)
 *   B3,B4  mirror   gen_rpm_ref
 *   B5     ctr+1    steal the counter the VCM is about to use (hi nibble 0)
 * DLC 6. B3/B4 and the stolen counter still make this byte-identical to the
 * VCM's next genuine frame whenever the VCM is also in 0x08, which is what
 * makes a counter-validating GENE MCU reject the VCM's real command as a
 * duplicate and accept only our zero.
 *
 * B0 is HELD at the engine-off state, not mirrored (spec 4.1, changed
 * 2026-09-19). Mirroring meant that when the VCM asked for a start it sent
 * 0x0B, we echoed 0x0B with zero torque, and the inverter drew ~10x its
 * engine-off power for the whole inhibit: gen_power median -0.11 kW with
 * excursions to -2.89 kW, against -0.01 kW engine-off. The VCM itself never
 * sustains (0x0B, zero torque) -- the longest such run in the corpus is
 * 0.19 s, a transition it passes through -- so the only sustained instance of
 * that combination anywhere was our own inhibit.
 *
 * Never transmit 0x10: that value is the VCM commanding generator shutdown,
 * and asserting it is the opposite of what this does. Holding a constant 0x08
 * makes that structurally impossible here.
 */
static void build_inhibit(const uint8_t *rx, int64_t t_rx, gi_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->id = GI_VCM_ID;      /* 0x051 -- the real command ID */
    f->dlc = 6;
    f->kind = GI_TX_INHIBIT;

    f->data[0] = GI_B0_ENGINE_OFF;
    f->data[1] = 0x00;
    f->data[2] = 0x80;
    f->data[3] = rx[3];
    f->data[4] = rx[4];
    f->data[5] = (uint8_t)(((rx[5] & 0x0F) + 1) & 0x0F);

    /* Reactive trail: transmit immediately, no offset wait -- no due_us. */
    f->have_t_rx = true;
    f->t_rx = t_rx;
}

void gi_on_frame(gi_state_t *st, uint32_t id, uint8_t dlc, const uint8_t *data,
                 int64_t now, gi_emit_t *out, gi_events_t *ev)
{
    /* Watch for M mode / low SoC on every frame -- latches the disable. */
    disable_monitor(st, id, dlc, data, now, ev);

    /*
     * Spec 7: freshness and values for the interlock signals. Must run before
     * the 0x051 filter below -- 0x471, 0x054 and 0x617 are all "other frames"
     * as far as the inhibit is concerned, and they are exactly the ones the
     * safety trips depend on.
     */
    interlock_monitor(st, id, dlc, data, now);

    if (id != GI_VCM_ID)
    {
        st->other_frames++;
        return;
    }

    if (st->have_prev_051)
    {
        gi_hist_add(&st->rx_gap, (uint32_t)(now - st->prev_051));
    }
    st->have_prev_051 = true;
    st->prev_051 = now;

    /*
     * Spec 6.3: track the VCM's shutdown command live, on every 0x051.
     * Recomputed from the current frame rather than latched, so leaving 0x10
     * resumes transmission by itself with no clearing step. Reported on the
     * edge only -- an episode can last 561 s and a per-frame event at 100 Hz
     * would bury everything else.
     */
    if (dlc >= 1)
    {
        bool now_shutdown = (data[0] == GI_B0_SHUTDOWN);
        if (now_shutdown != st->shutdown_suppressed)
        {
            st->shutdown_suppressed = now_shutdown;
            ev_add(ev, now, GI_EV_SHUTDOWN_SUPPRESS,
                   now_shutdown ? 1 : 0, (int32_t)data[0], 0);
        }
    }

    /*
     * B5 is the VCM's rolling counter -- 6422 of 6422 steps were exactly +1
     * in the reference capture. Tracking it here is a cheap check that we are
     * seeing every frame rather than silently dropping some.
     */
    if (dlc >= 6)
    {
        uint8_t ctr = gi_ctr(data);
        if (st->have_last_ctr)
        {
            if (((st->last_ctr + 1) & 0x0F) == ctr) st->ctr_steps_ok++;
            else                                    st->ctr_steps_bad++;
        }
        st->last_ctr = ctr;
        st->have_last_ctr = true;
    }

    if (st->mode == GI_RESPOND)
    {
        gi_frame_t f;
        build_probe(st, now, &f);
        emit(out, &f);
    }
    else if (st->mode == GI_INHIBIT && !st->disabled
             && !st->shutdown_suppressed && st->inhibit_live)
    {
        gi_frame_t f;
        if (dlc < 6)
        {
            st->tx_fail++;
            return;
        }
        build_inhibit(data, now, &f);
        emit(out, &f);
    }
}

void gi_on_tx_result(gi_state_t *st, const gi_frame_t *f, bool ok,
                     int64_t t_tx, gi_events_t *ev)
{
    if (f->kind == GI_TX_DIAG)
    {
        /* Best-effort; fails silently in listen-only, and is not counted. */
        return;
    }

    if (ok)
    {
        st->tx_ok++;
        if (f->have_t_rx)
        {
            gi_hist_add(&st->response, (uint32_t)(t_tx - f->t_rx));
        }
        return;
    }

    st->tx_fail++;
    ev_add(ev, t_tx, GI_EV_TX_FAIL, (int32_t)f->kind, 0, 0);

    /*
     * Spec 7: abort immediately on a failed transmit. Unlike an error frame,
     * which is a property of the bus and is therefore judged on a rate, a
     * failed transmit is unambiguously OURS -- our frame did not go out, so
     * the VCM's command stands and we are not inhibiting anything. There is
     * nothing to average over.
     *
     * Only for the inhibit frame: a dropped timing probe costs a measurement,
     * not a safety property, and the pre-refactor code did not abort on it.
     */
    if (f->kind == GI_TX_INHIBIT)
    {
        inhibit_abort(st, GI_ABORT_TX_FAILED, t_tx, ev);
    }
}

bool gi_on_rx_error(gi_state_t *st, int64_t now, gi_events_t *ev)
{
    st->rx_errors++;
    if (st->rx_errors >= st->cfg.max_rx_errors)
    {
        ev_add(ev, now, GI_EV_RX_ERROR_DISARM, (int32_t)st->rx_errors, 0, 0);
        st->mode = GI_OFF;
        st->rx_errors = 0;
        return true;
    }
    return false;
}

void gi_on_rx_ok(gi_state_t *st)
{
    st->rx_errors = 0;
}
