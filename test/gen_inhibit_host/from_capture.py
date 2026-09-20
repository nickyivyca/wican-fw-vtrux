#!/usr/bin/env python3
"""Turn a real CAN capture into a host_runner scenario.

This is the half of the harness that is worth more than the synthetics. The
synthetic scenarios test the states I thought of; a real capture replays the
frame orderings the truck actually produces, including the ones nobody would
invent -- and arrival order is precisely where the bugs in this component
have lived.

  --scan          list key-on candidates in a capture (a silence, then 0x051)
  --at T --for D  emit a scenario covering [T, T+D) with INHIBIT armed at T

TIMEBASE. canre yields seconds; the runner wants integer microseconds, and
scenario time is rebased so the window starts at 0. That rebase is exactly
why the core must not use timestamp 0 as a "never seen" sentinel -- see the
note at the top of gen_inhibit_core.h.

  BUSMASTER data-line timestamps are 10 kHz ticks, not milliseconds. canre's
  parser already handles that; this script must not re-scale.

WHAT A REPLAY PROVES, AND WHAT IT DOES NOT. It proves the decision sequence
is stable against real traffic. It cannot prove latency -- every transmit
here is instantaneous, and the response histogram is meaningless. It also
cannot see anything the capture did not contain, which for most captures
means no generator activity at all.
"""

import argparse
import os
import sys

# The parsers live in the project repo, which is a SEPARATE tree from this
# firmware repo.
DEFAULT_REPO = os.environ.get(
    "GEN_INHIBIT_REPO", os.path.expanduser("~/Seafile/NotGit/reverse-it"))

CMD = 0x051
RPM = 0x054
FB = 0x471
SOC = 0x411
SHIFT = 0x639
FAULT = 0x617
RELEVANT = (CMD, RPM, FB, SOC, SHIFT, FAULT)


def load_parser(repo):
    sys.path.insert(0, repo)
    from canre.parsers import parse_file
    return parse_file


def frames(parse_file, path, ids=None, t0=None, t1=None):
    """Yield (t_seconds, id, dlc, bytes), skipping untimestamped frames."""
    for fr in parse_file(path):
        if fr.timestamp is None:
            continue
        if t0 is not None and fr.timestamp < t0:
            continue
        if t1 is not None and fr.timestamp >= t1:
            break
        ident = fr.arbitration_id          # NOT .can_id -- that attribute
                                           # does not exist and a broad except
                                           # once turned the AttributeError
                                           # into "0 logs scanned".
        if ids and ident not in ids:
            continue
        data = bytes(fr.data or b"")
        yield fr.timestamp, ident, len(data), data


def do_scan(parse_file, path, gap):
    """Find points where 0x051 resumes after a silence -- i.e. key-on."""
    prev = None
    first = None
    hits = []
    n = 0
    for t, ident, _dlc, _d in frames(parse_file, path, ids=(CMD,)):
        n += 1
        if first is None:
            first = t
        if prev is not None and (t - prev) > gap:
            hits.append((prev, t, t - prev))
        prev = t

    print("%s" % os.path.basename(path))
    print("  0x051 frames: %d" % n)
    if first is None:
        print("  no 0x051 at all -- not a powertrain-bus capture")
        return
    print("  first 0x051 at t=%.3f s, last at t=%.3f s" % (first, prev))
    if not hits:
        print("  no gap longer than %.1f s -- continuous; use --at %.3f"
              % (gap, first))
        return
    print("  %d wake events (gap > %.1f s):" % (len(hits), gap))
    for a, b, d in hits[:20]:
        print("    silence %8.3f -> %8.3f  (%6.1f s)   try --at %.3f"
              % (a, b, d, b - 0.5))


def do_emit(parse_file, path, at, dur, out, ids, arm_at, mode):
    t0, t1 = at, at + dur
    lines = []
    kept = 0
    counts = {}
    for t, ident, dlc, data in frames(parse_file, path, ids=ids, t0=t0, t1=t1):
        us = int(round((t - t0) * 1e6))
        lines.append("f %d %03X %d %s" % (us, ident, dlc, data.hex().upper()))
        counts[ident] = counts.get(ident, 0) + 1
        kept += 1

    if not kept:
        print("no frames in [%.3f, %.3f) -- wrong window?" % (t0, t1))
        return 1

    arm_us = int(round(arm_at * 1e6))
    header = [
        "# replay of %s" % os.path.basename(path),
        "# window [%.3f, %.3f) s of the capture, rebased to 0" % (t0, t1),
        "# %d frames; per-ID counts: %s" % (
            kept, ", ".join("0x%03X=%d" % (i, c)
                            for i, c in sorted(counts.items()))),
        "#",
        "# Real traffic. What this pins is the DECISION SEQUENCE against"
        " orderings",
        "# the truck actually produced. It says nothing about latency: every"
        " transmit",
        "# in the harness is instantaneous.",
        "#",
        "mode %d %d 500" % (arm_us, mode),
    ]
    with open(out, "w", newline="\n") as fh:
        fh.write("\n".join(header) + "\n")
        fh.write("\n".join(lines) + "\n")
        fh.write("end %d\n" % (int(round(dur * 1e6)) + 1000000))

    print("wrote %s -- %d frames" % (out, kept))
    for i, c in sorted(counts.items()):
        print("    0x%03X  %6d" % (i, c))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--repo", default=DEFAULT_REPO)
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--gap", type=float, default=5.0,
                    help="scan: silence that counts as a wake event")
    ap.add_argument("--at", type=float, help="window start, seconds into log")
    ap.add_argument("--for", dest="dur", type=float, default=60.0)
    ap.add_argument("--arm-at", type=float, default=0.0,
                    help="seconds into the WINDOW at which to arm")
    ap.add_argument("--mode", type=int, default=3, choices=(0, 1, 2, 3))
    ap.add_argument("--all-ids", action="store_true",
                    help="keep every frame, not just the six the core reads."
                         " Slower and larger, but other_frames then means"
                         " what it means on the device.")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    parse_file = load_parser(args.repo)

    if args.scan:
        do_scan(parse_file, args.log, args.gap)
        return 0

    if args.at is None:
        print("need --at (use --scan to find a key-on)")
        return 2

    out = args.out or os.path.join(
        "scenarios",
        "replay-%s-%.0f.scn" % (
            os.path.splitext(os.path.basename(args.log))[0][:40], args.at))
    ids = None if args.all_ids else RELEVANT
    return do_emit(parse_file, args.log, args.at, args.dur, out, ids,
                   args.arm_at, args.mode)


if __name__ == "__main__":
    sys.exit(main())
