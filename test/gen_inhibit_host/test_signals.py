#!/usr/bin/env python3
"""Cross-check the core's hand-rolled extractors against cantools.

WHY THIS TEST IS NOT OPTIONAL

gen_inhibit_core.c hand-rolls bit extraction, against AGENTS.md's standing
rule to decode with cantools and never hand-roll. It has to: the core runs on
an ESP32 with no DBC and no cantools. But that means every extractor is a
hand TRANSCRIPTION of an external artifact, and a transcription error is
invisible to every other test in this suite -- the scenarios, the goldens and
the replay would all agree happily with a wrong value, because they all get
their value from the same wrong line of C.

This project already has one live instance of exactly that failure. The
truck-validated Python reference decodes GENE_RotSpd with a generic helper
that subtracts 32768, where the DBC says 32767. Immaterial against a 300 rpm
threshold, and nobody re-checked it for years.

So: 2000 randomised frames per signal, C against cantools, exact equality.
The idea and the frame count are lifted from the interposer tree's own
test_signals.py, which does this for its extractors.

WHAT A FAILURE MEANS. The DBC is the source of truth for encoding
(AGENTS.md). A mismatch means the C is wrong, unless the DBC itself has been
edited -- in which case stop and raise it, do not "fix" the C to match a
change nobody reviewed.
"""

import argparse
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = os.path.join(HERE, "extract_probe")

# The project repo is a SEPARATE tree from this firmware repo. Default to the
# layout on madhouse-debian, override with --repo or GEN_INHIBIT_REPO.
DEFAULT_REPO = os.environ.get(
    "GEN_INHIBIT_REPO", os.path.expanduser("~/Seafile/NotGit/reverse-it"))

# (kind, dbc file, message name, [signal names the C returns, in order])
CASES = [
    ("cmd",   "vtrux-powertrain-experimental.dbc", "VCM_GenControl_0051",
     ["gen_torque_cmd", "gen_rpm_ref"]),
    ("rpm",   "epri-pt-bus.dbc", "EPRI_MCU_Cmd2_Response_0054",
     ["GENE_RotSpd"]),
    ("soc",   "vtrux-powertrain-experimental.dbc", "BMS_SoC_HiRes_0411",
     ["bms_soc_hires"]),
    ("shift", "vtrux-powertrain-experimental.dbc", "VCM_ShiftPos_0639",
     ["shift_lever_pos"]),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=DEFAULT_REPO)
    ap.add_argument("-n", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=20260919)
    args = ap.parse_args()

    try:
        import cantools
    except ImportError:
        print("cantools not installed; cannot cross-check. pip install cantools")
        return 2

    if not os.path.exists(PROBE):
        print("build extract_probe first (make)")
        return 2

    dbcdir = os.path.join(args.repo, "projects", "vtrux")
    rng = random.Random(args.seed)
    total_fail = 0

    for kind, dbcfile, msgname, signames in CASES:
        path = os.path.join(dbcdir, dbcfile)
        if not os.path.exists(path):
            print("[SKIP  ] %-6s %s not found" % (kind, path))
            continue
        db = cantools.database.load_file(path)
        try:
            msg = db.get_message_by_name(msgname)
        except KeyError:
            print("[SKIP  ] %-6s %s has no message %s" % (kind, dbcfile, msgname))
            continue

        # Random payloads of the message's own length, so cantools decodes
        # them and the C sees exactly the same bytes.
        frames = [bytes(rng.randrange(256) for _ in range(msg.length))
                  for _ in range(args.n)]

        stdin = "".join("%s %s\n" % (kind, f.hex()) for f in frames)
        r = subprocess.run([PROBE], input=stdin, capture_output=True, text=True)
        if r.returncode != 0:
            print("[ERROR ] %-6s probe exited %d: %s" % (kind, r.returncode,
                                                         r.stderr[:200]))
            total_fail += 1
            continue
        got = r.stdout.strip().splitlines()
        if len(got) != len(frames):
            print("[ERROR ] %-6s probe returned %d lines for %d frames"
                  % (kind, len(got), len(frames)))
            total_fail += 1
            continue

        nbad = 0
        first = None
        for f, line in zip(frames, got):
            dec = msg.decode(f, decode_choices=False, allow_truncated=True)
            cvals = line.split()

            want = []
            for sn in signames:
                v = dec[sn]
                # bms_soc_hires is the one scaled signal; the C returns the
                # raw count (percent * 100), so undo the 0.01 factor. Round
                # rather than int() -- cantools returns a Decimal/float and
                # truncation would disagree on exact halves.
                if sn == "bms_soc_hires":
                    v = round(float(v) * 100)
                want.append(int(v))

            # The counter is a C-only extra on 'cmd'; check it against the
            # raw byte rather than the DBC, which does not define it.
            gotv = [int(x) for x in cvals[:len(signames)]]
            if kind == "cmd":
                if int(cvals[2]) != (f[5] & 0x0F):
                    nbad += 1
                    if first is None:
                        first = (f.hex(), "ctr", cvals[2], f[5] & 0x0F)
                    continue

            if gotv != want:
                nbad += 1
                if first is None:
                    first = (f.hex(), signames, gotv, want)

        if nbad:
            print("[FAIL  ] %-6s %s: %d/%d mismatched" % (kind, msgname, nbad,
                                                          len(frames)))
            print("         first: frame=%s signal=%s C=%s cantools=%s" % first)
            total_fail += 1
        else:
            print("[ok    ] %-6s %-34s %d frames, %s"
                  % (kind, msgname, len(frames), ", ".join(signames)))

    print()
    if total_fail:
        print("%d signal group(s) FAILED -- the DBC is the source of truth for"
              " encoding; do not change the C to match an unreviewed DBC edit"
              % total_fail)
        return 1
    print("all extractors agree with cantools")
    return 0


if __name__ == "__main__":
    sys.exit(main())
