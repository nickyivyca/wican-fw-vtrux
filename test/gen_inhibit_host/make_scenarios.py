#!/usr/bin/env python3
"""Generate the synthetic scenario files the host runner replays.

These are SYNTHETIC. They exist for the states the real corpus does not
contain -- a failed transmit, an error-frame storm, an inverter that drops off
a live bus -- and for the three bugs found by hand on 2026-09-19, which are
what this harness exists to keep fixed:

  bug 1  the arm gate required fresh 0x471/0x054, so the inhibit would have
         gone live ~28 s after key-on or never       -> keyon-gene-late
  bug 2  the 0x471 trip fired without ever having seen 0x471, so it aborted
         instantly on every normal key-on            -> keyon-normal
  bug 3  inhibit_live survived an ordinary disarm    -> disarm-clears-live

A scenario is a plain text file of directives; see host_runner.c for the
grammar. Frame rates here are chosen to keep traces small and are NOT claims
about the truck -- the real rates come from replaying real captures, which is
what from_capture.py is for.

Nothing here asserts. The runner prints a trace, run_tests.py diffs it against
a golden. What each scenario is meant to demonstrate is in its header comment,
which is carried into the scenario file so a failing diff explains itself.
"""

import argparse
import os

US = 1
MS = 1000
S = 1000000


# --------------------------------------------------------------- frames --

def _f(t, ident, data, dlc=None):
    if dlc is None:
        dlc = len(data)
    hexs = "".join("%02X" % b for b in data)
    return "f %d %03X %d %s" % (t, ident, dlc, hexs)


def cmd(t, ctr, b0=0x08, torque=0, rpm_ref=-1):
    """0x051 VCM generator command, DLC 6.

    B1-B2 gen_torque_cmd LE offset -32768; B3-B4 gen_rpm_ref same;
    B5 low nibble is the rolling counter.
    """
    tq = torque + 32768
    rr = rpm_ref + 32768
    return _f(t, 0x051, [b0, tq & 0xFF, (tq >> 8) & 0xFF,
                         rr & 0xFF, (rr >> 8) & 0xFF, ctr & 0x0F])


def gene(t, rpm):
    """0x054 GENE_RotSpd, LE, zero at 32767 per the DBC (not 32768)."""
    v = rpm + 32767
    return _f(t, 0x054, [v & 0xFF, (v >> 8) & 0xFF, 0, 0, 0, 0, 0, 0])


def fb(t):
    """0x471 inverter feedback. Liveness only; the payload is not read."""
    return _f(t, 0x471, [0] * 8)


def soc(t, raw):
    """0x411 BMS_SoC_HiRes: 14-bit big-endian at bit 7, scale 0.01%."""
    return _f(t, 0x411, [(raw >> 6) & 0xFF, (raw & 0x3F) << 2, 0, 0, 0, 0, 0, 0])


def shift(t, pos):
    """0x639 shift_lever_pos in B6 bits 6:4. 4 = Manual_generator_mode."""
    d = [0] * 8
    d[6] = (pos & 0x07) << 4
    return _f(t, 0x639, d)


def fault(t, val):
    """0x617 B7: 0xCA = VCM fault active, 0xC8 = clear."""
    d = [0] * 8
    d[7] = val
    return _f(t, 0x617, d)


def cmd_train(t0, t1, period, ctr0=0, **kw):
    """A run of 0x051 at a fixed period, counter incrementing mod 16."""
    out = []
    ctr = ctr0
    t = t0
    while t < t1:
        out.append(cmd(t, ctr, **kw))
        ctr = (ctr + 1) & 0x0F
        t += period
    return out


def periodic(t0, t1, period, fn, phase=0):
    out = []
    t = t0 + phase
    while t < t1:
        out.append(fn(t))
        t += period
    return out


# ------------------------------------------------------------ scenarios --

SCEN = {}


def scenario(name, why):
    def deco(fn):
        SCEN[name] = (why, fn)
        return fn
    return deco


@scenario("keyon-normal", """
Ordinary key-on with an auto-arm build. The bus is silent, the device arms
INHIBIT with nothing on the wire, then the VCM starts talking.

EXPECT: blocked on "no fresh 0x051" while silent; goes live within a frame or
two of the bus waking; transmits a held 0x08 zero-torque 0x051 per received
frame; NEVER aborts. Pins bug 2 -- the 0x471 trip must not fire on a bus whose
inverter has not powered up, because never-seen is not the same as stopped.
""")
def s_keyon_normal():
    L = ["mode 0 3 500"]              # auto-arm INHIBIT at t=0, bus silent
    L += cmd_train(3 * S, 8 * S, 20 * MS)
    L += ["end %d" % (9 * S)]
    return L


@scenario("keyon-gene-late", """
Key-on where the GENE family lags the VCM by 28 s, which is what the corpus
actually shows (0x051 at +0.17 s, 0x471/0x054 at +28 s or never).

EXPECT: live within a frame or two of 0x051 starting -- NOT 28 s later. Pins
bug 1. Once 0x471 does appear, fb_ever latches and the inverter-lost trip
becomes armed for the rest of the run; the trace must show that transition and
still no abort while 0x471 keeps arriving.
""")
def s_keyon_gene_late():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 40 * S, 50 * MS)
    L += periodic(29 * S, 40 * S, 100 * MS, fb)
    L += periodic(29 * S, 40 * S, 100 * MS, lambda t: gene(t, 0))
    L += ["end %d" % (41 * S)]
    return sorted_directives(L)


@scenario("disarm-clears-live", """
Arm, go live, then an ordinary disarm (mode 0) -- not an abort.

EXPECT: inhibit_live returns to 0 and arm_block becomes "not armed" the moment
the worker next reaches the OFF branch. Pins bug 3: leaving the flag set made
diag_flags bit5 and the JSON claim a live inhibit on a disarmed device, which
is exactly the reassuring-but-wrong reading those flags exist to prevent.
""")
def s_disarm_clears_live():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS)
    L += ["mode %d 0 500" % (3 * S)]
    L += cmd_train(3 * S + 100 * MS, 4 * S, 20 * MS)   # dropped while OFF
    L += ["end %d" % (5 * S)]
    return sorted_directives(L)


@scenario("b0-held-not-mirrored", """
The VCM asks for a start: B0 goes 0x08 -> 0x0B with nonzero torque.

EXPECT: every transmitted frame keeps B0 = 0x08 and B1-B2 = 00 80. Spec 4.1,
changed 2026-09-19 -- mirroring 0x0B made the inverter draw ~10x its
engine-off power for the whole inhibit. B3-B4 still mirror gen_rpm_ref and B5
is still the stolen counter+1, so the frame stays byte-identical to the VCM's
next genuine frame whenever the VCM is also in 0x08.
""")
def s_b0_held():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 2 * S, 20 * MS)
    # VCM requests a start: B0=0x0B, torque nonzero, rpm_ref positive.
    L += cmd_train(2 * S, 3 * S, 20 * MS, ctr0=0, b0=0x0B,
                   torque=400, rpm_ref=1200)
    L += ["end %d" % (4 * S)]
    return sorted_directives(L)


@scenario("shutdown-suppress", """
The VCM parks the generator: 0x051 B0 = 0x10 for a spell, then back to 0x08.

EXPECT: transmission stops for the whole 0x10 episode and resumes by itself
afterwards with no clearing step. NOT latched, deliberately -- 0x10 is a state
the truck passes through and drives out of again (40,697 frames of it, 100%
with the shifter in Park, up to four episodes in one capture), and latching
would stand the inhibitor down for the drive it exists for.
""")
def s_shutdown_suppress():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 2 * S, 20 * MS)
    L += cmd_train(2 * S, 4 * S, 20 * MS, b0=0x10)
    L += cmd_train(4 * S, 5 * S, 20 * MS)
    L += ["end %d" % (6 * S)]
    return sorted_directives(L)


@scenario("shutdown-at-keyon", """
0x10 is the normal state at key-on, so the device arms straight into it.

EXPECT: the gate blocks on "VCM commanding 0x10" rather than going live and
sitting silent, and it clears by itself when the VCM leaves 0x10. This is why
the suppression needs no startup guard.
""")
def s_shutdown_at_keyon():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS, b0=0x10)
    L += cmd_train(3 * S, 4 * S, 20 * MS)
    L += ["end %d" % (5 * S)]
    return sorted_directives(L)


@scenario("m-mode-latch", """
Driver engages M mode (0x639 shift_lever_pos == 4) mid-run, then leaves it.

EXPECT: disable latches immediately and NEVER clears -- no transmission for
the rest of the run even after the shifter moves away from 4. Cleared only by
a reboot, which the relay dropping at truck sleep provides.
""")
def s_m_mode_latch():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 6 * S, 20 * MS)
    L += periodic(1 * S, 3 * S, 200 * MS, lambda t: shift(t, 2))
    L += periodic(3 * S, 4 * S, 200 * MS, lambda t: shift(t, 4))
    L += periodic(4 * S, 6 * S, 200 * MS, lambda t: shift(t, 2))
    L += ["end %d" % (7 * S)]
    return sorted_directives(L)


@scenario("low-soc-debounce", """
SoC crosses the 21% floor. Three separate things are being pinned.

EXPECT: (a) raw 0 is the startup sentinel and never counts as 0% -- no latch
however many arrive; (b) four consecutive sub-threshold samples do not latch;
(c) a sample above the floor resets the count, so 4-high-4 does not latch
either; (d) the fifth consecutive sub-threshold sample latches low_soc.
""")
def s_low_soc():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 12 * S, 20 * MS)
    t = 1 * S
    for _ in range(6):                      # (a) sentinel zeros
        L.append(soc(t, 0)); t += 200 * MS
    for _ in range(4):                      # (b) four low, no latch
        L.append(soc(t, 2050)); t += 200 * MS
    L.append(soc(t, 2500)); t += 200 * MS   # (c) one high resets the count
    for _ in range(4):
        L.append(soc(t, 2050)); t += 200 * MS
    L.append(soc(t, 2500)); t += 200 * MS
    for _ in range(5):                      # (d) five consecutive -> latch
        L.append(soc(t, 2050)); t += 200 * MS
    L += periodic(t, 12 * S, 200 * MS, lambda tt: soc(tt, 2050))
    L += ["end %d" % (13 * S)]
    return sorted_directives(L)


@scenario("bus-loss-latches", """
A live inhibit loses the CAN link: 0x051 stops for longer than the freshness
window, then comes back.

EXPECT: abort "bus lost -- no 0x051 (latched; CAN link down)", mode drops to
OFF, and -- the point of the scenario -- it does NOT re-arm when the frames
return. Latching is deliberate (2026-09-19): a bus that drops and returns is
an unreliable environment, and this device steals the VCM's rolling counter
and transmits a real 0x051 onto a live powertrain bus. Resuming across a link
we already have evidence is unsound would mean doing that repeatedly, through
a gate whose freshness checks a flapping bus can satisfy.
""")
def s_bus_loss():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS)
    # 1.5 s of nothing -- three freshness windows.
    L += cmd_train(4500 * MS, 8 * S, 20 * MS)
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("bus-glitch-short", """
The same shape as bus-loss-latches but the gap is 0.3 s -- INSIDE the 0.5 s
freshness window.

EXPECT: no abort at all. This is the scenario that says the freshness window
is doing its job rather than the trip being hair-triggered, and it is the one
that would catch a careless tightening of fresh_us.
""")
def s_bus_glitch():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS)
    L += cmd_train(3300 * MS, 6 * S, 20 * MS)
    L += ["end %d" % (7 * S)]
    return sorted_directives(L)


@scenario("inverter-lost", """
0x471 has been arriving, then stops while 0x051 keeps running at full rate.

EXPECT: abort "0x471 stopped while 0x051 still live -- inverter lost". This is
the case the trip is actually for, and it is only reachable once 0x471 has
been heard -- the distinction the laptop tool could not make, and the one that
bug 2 got wrong in the other direction.
""")
def s_inverter_lost():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 8 * S, 20 * MS)
    L += periodic(1 * S, 4 * S, 100 * MS, fb)
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("rpm-debounce", """
GENE rpm crosses the 300 threshold twice: once for 0.2 s, once for 0.5 s.

EXPECT: the 0.2 s excursion does NOT trip; the 0.5 s one does, with
"engine turning while armed -- the inhibit did not hold". The debounce is not
optional -- a momentary crank blip is exactly what a WORKING inhibit produces,
and aborting on it would end the run the inhibit just won.
""")
def s_rpm_debounce():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 10 * S, 20 * MS)
    L += periodic(1 * S, 3 * S, 100 * MS, lambda t: gene(t, 0))
    L += periodic(3 * S, 3200 * MS, 50 * MS, lambda t: gene(t, 450))
    L += periodic(3200 * MS, 6 * S, 100 * MS, lambda t: gene(t, 0))
    L += periodic(6 * S, 6500 * MS, 50 * MS, lambda t: gene(t, 450))
    L += periodic(6500 * MS, 10 * S, 100 * MS, lambda t: gene(t, 0))
    L += ["end %d" % (11 * S)]
    return sorted_directives(L)


@scenario("arm-gate-generator-running", """
The device arms while the generator is already turning at 800 rpm.

EXPECT: blocked on "generator running" -- a block, not an abort, so it can
still go live later if the generator stops. Taking over a loaded generator and
commanding zero would shed the engine's whole load in one frame.
""")
def s_gate_gen_running():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 8 * S, 20 * MS)
    L += periodic(1 * S, 4 * S, 100 * MS, lambda t: gene(t, 800))
    L += periodic(4 * S, 8 * S, 100 * MS, lambda t: gene(t, 0))
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("arm-gate-order-rpm-first", """
FINDING (2026-09-19, found by this harness). Pairs with
arm-gate-order-cmd-first: the two differ by ONE MICROSECOND in arrival order
and produce opposite outcomes.

Here 0x054 (800 rpm, generator running) arrives just BEFORE the first 0x051.

EXPECT, and currently observed: the gate sees a fresh 0x054 above the
threshold and blocks on "generator running". Nothing is ever transmitted.
This is the intended behaviour.
""")
def s_gate_order_rpm_first():
    L = ["mode 0 3 500"]
    L += periodic(1 * S, 4 * S, 100 * MS, lambda t: gene(t, 800))
    L += cmd_train(1 * S + 1, 4 * S, 20 * MS)
    L += ["end %d" % (5 * S)]
    return sorted_directives(L)


@scenario("arm-gate-order-cmd-first", """
FINDING (2026-09-19, found by this harness). The same situation as
arm-gate-order-rpm-first with the first two frames swapped: 0x051 arrives one
microsecond BEFORE the first 0x054.

CURRENTLY OBSERVED: the gate runs before any 0x054 has been seen, so
gi_fresh(have_rpm) is false, the generator-running check is SKIPPED, and the
inhibit goes live. It then transmits ~16 zero-torque 0x051 frames at a
generator turning 800 rpm before the runtime rpm trip's 0.3 s debounce ends
the run.

This is pre-refactor behaviour, faithfully preserved -- it is NOT a
regression introduced by the core split. It is recorded here as a golden so
that the behaviour is visible and any change to it shows up as a diff.

WHY IT MATTERS: the arm gate's stated purpose for this check is that "taking
over a loaded generator and commanding zero sheds the engine's whole load in
one frame -- a load dump on a running engine". Whether that protection holds
currently depends on which of two frames arrives first. Raised with the user;
the fix belongs in the spec before it belongs in the code.
""")
def s_gate_order_cmd_first():
    L = ["mode 0 3 500"]
    L += periodic(1 * S + 1, 4 * S, 100 * MS, lambda t: gene(t, 800))
    L += cmd_train(1 * S, 4 * S, 20 * MS)
    L += ["end %d" % (5 * S)]
    return sorted_directives(L)


@scenario("disable-freezes-live-flag", """
FINDING (2026-09-19, found by this harness). A latched section 6 disable
(here M mode) arrives while the inhibit is live.

CURRENTLY OBSERVED: transmission stops immediately, correctly. But
inhibit_live stays 1 for the rest of the run, because the whole interlock
block is guarded by `mode == INHIBIT && !disabled`, so once disabled neither
the runtime trips nor the arm gate run again, and nothing clears the flag.
The worker never reaches its OFF branch either, since the mode is still
INHIBIT.

Consequence: diag_flags bit5 and the JSON "inhibit_live" report a live
inhibit on a device that is latched off and transmitting nothing.

This is the SAME defect class as the bug fixed on 2026-09-19 where
inhibit_live survived an ordinary disarm -- "exactly the kind of
reassuring-but-wrong reading these flags exist to prevent". Pre-refactor
behaviour, preserved deliberately; raised with the user rather than fixed
here, because the spec is the source of truth for what the flag means.
""")
def s_disable_freezes_live():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 6 * S, 20 * MS)
    L += periodic(1 * S, 3 * S, 200 * MS, lambda t: shift(t, 2))
    L += periodic(3 * S, 6 * S, 200 * MS, lambda t: shift(t, 4))
    L += ["end %d" % (7 * S)]
    return sorted_directives(L)


@scenario("arm-gate-vcm-wants-engine", """
The VCM is requesting the engine when the device arms (rpm_ref >= 0, nonzero
torque), then stops requesting.

EXPECT: blocked on "VCM requesting engine", then on nothing, then live. The
only transition the gate arms into is "engine off, VCM tries to start it, we
stop it".
""")
def s_gate_vcm_wants():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS, b0=0x0B, torque=300, rpm_ref=900)
    L += cmd_train(3 * S, 5 * S, 20 * MS)
    L += ["end %d" % (6 * S)]
    return sorted_directives(L)


@scenario("vcm-fault", """
0x617 B7 goes to 0xCA (fault active) while live, then clears.

EXPECT: abort "0x617 B7 = 0xCA (VCM fault active)". Once aborted the mode is
OFF and the clearing frame does not bring it back -- an abort is not a block.
""")
def s_vcm_fault():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 8 * S, 20 * MS)
    L += periodic(1 * S, 3 * S, 200 * MS, lambda t: fault(t, 0xC8))
    L += periodic(3 * S, 5 * S, 200 * MS, lambda t: fault(t, 0xCA))
    L += periodic(5 * S, 8 * S, 200 * MS, lambda t: fault(t, 0xC8))
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("tx-fail-aborts", """
Transmits start failing while the inhibit is live.

EXPECT: abort "transmit failed" on the FIRST failure, no averaging. Unlike an
error frame, which is a property of the bus and is judged on a rate, a failed
transmit is unambiguously ours -- our frame did not go out, so the VCM's
command stands and we are not inhibiting anything.
""")
def s_tx_fail():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 6 * S, 20 * MS)
    L += ["txfail %d %d" % (3 * S, 6 * S)]
    L += ["end %d" % (7 * S)]
    return sorted_directives(L)


@scenario("err-rate-trip", """
The controller's bus_error_count climbs by 12 inside one 10 s window.

EXPECT: abort "error-frame rate exceeded" once the delta reaches 10. Rate
based, never first-strike: aborting on the first error frame ended three
consecutive armed runs, and the running-generator regime legitimately bursts
to 4 in any 10 s.
""")
def s_err_rate():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 8 * S, 20 * MS)
    for i in range(1, 13):
        L.append("bus %d 1 1 1 %d" % (2 * S + i * 100 * MS, i))
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("err-rate-under", """
The same climb but stopping at 9 within the window.

EXPECT: no abort. Pins the threshold from the other side -- 10 in 10 s keeps
2.5x headroom over the worst observed burst, and this is the scenario that
fails if someone lowers it.
""")
def s_err_under():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 8 * S, 20 * MS)
    for i in range(1, 10):
        L.append("bus %d 1 1 1 %d" % (2 * S + i * 100 * MS, i))
    L += ["end %d" % (9 * S)]
    return sorted_directives(L)


@scenario("observe-never-transmits", """
Mode OBSERVE with a fully live bus.

EXPECT: no PROBE and no INHIBIT frames. Diag frames ARE emitted by the core --
on the device they then fail in listen-only, which is the documented cost of a
genuinely passive tap, and is a shim behaviour this harness cannot see.
""")
def s_observe():
    L = ["mode 0 1 500"]
    L += cmd_train(1 * S, 4 * S, 20 * MS)
    L += periodic(1 * S, 4 * S, 100 * MS, fb)
    L += ["end %d" % (5 * S)]
    return sorted_directives(L)


@scenario("respond-probe-offset", """
Mode RESPOND with a 500 us probe offset.

EXPECT: one 0x7F0 per received 0x051, due 500 us after the receive, echoing
the RX timestamp in B0-B3 and the offset in B4-B5. Never 0x051.
""")
def s_respond():
    L = ["mode 0 2 500"]
    L += cmd_train(1 * S, 2 * S, 20 * MS)
    L += ["end %d" % (3 * S)]
    return sorted_directives(L)


@scenario("rearm-after-disarm", """
Arm, go live, disarm, arm again on a still-live bus.

EXPECT: the second arm goes live again promptly -- signal freshness is
deliberately NOT cleared by the arm-cycle reset, because it is a property of
the bus and not of the run, and clearing it would make every arm wait a fresh
round. Statistics DO reset. fb_ever resets, so the inverter-lost trip re-arms
from scratch.
""")
def s_rearm():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 3 * S, 20 * MS)
    L += periodic(1 * S, 6 * S, 100 * MS, fb)
    L += ["mode %d 0 500" % (3 * S)]
    L += ["mode %d 3 500" % (3500 * MS)]
    L += cmd_train(3500 * MS, 6 * S, 20 * MS)
    L += ["end %d" % (7 * S)]
    return sorted_directives(L)


@scenario("short-dlc-0x051", """
A 0x051 arrives with DLC 4 -- too short to rebuild an inhibit frame from.

EXPECT: no transmit, tx_fail increments, no abort. Also confirms the
interlock monitor's own DLC guards hold: torque and rpm_ref need DLC >= 5 and
must keep their previous values rather than read past the end.
""")
def s_short_dlc():
    L = ["mode 0 3 500"]
    L += cmd_train(1 * S, 2 * S, 20 * MS)
    L += [_f(2 * S + i * 20 * MS, 0x051, [0x08, 0x00, 0x80, 0x00], 4)
          for i in range(25)]
    L += cmd_train(2500 * MS, 3500 * MS, 20 * MS)
    L += ["end %d" % (4 * S)]
    return sorted_directives(L)


# ------------------------------------------------------------- plumbing --

def sorted_directives(lines):
    """Stable-sort directive lines by their timestamp field.

    Scenarios are built by concatenating independent frame trains, so they
    arrive interleaved out of order. The runner requires ascending time. Sort
    is stable, so frames written at the same timestamp keep the order the
    scenario declared -- which is itself part of what a trace pins.
    """
    def key(item):
        i, ln = item
        p = ln.split()
        if p[0] in ("f", "mode", "bus", "txfail", "end"):
            return (int(p[1]), i)
        return (-1, i)
    return [ln for _, ln in sorted(enumerate(lines), key=key)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="scenarios")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for name in sorted(SCEN):
            why = SCEN[name][0].strip().splitlines()[0]
            print("%-28s %s" % (name, why))
        return

    os.makedirs(args.out, exist_ok=True)
    for name in sorted(SCEN):
        why, fn = SCEN[name]
        lines = fn()
        path = os.path.join(args.out, name + ".scn")
        with open(path, "w", newline="\n") as fh:
            fh.write("# %s\n#\n" % name)
            for ln in why.strip().splitlines():
                fh.write("# %s\n" % ln)
            fh.write("#\n")
            for ln in lines:
                fh.write(ln + "\n")
        print("wrote %-40s %5d lines" % (path, len(lines)))


if __name__ == "__main__":
    main()
