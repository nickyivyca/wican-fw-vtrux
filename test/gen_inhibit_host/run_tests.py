#!/usr/bin/env python3
"""Build the host runner, replay every scenario, diff against the goldens.

  ./run_tests.py            build, run, diff, report
  ./run_tests.py --bless    overwrite the goldens with current output
  ./run_tests.py --only X   just the scenarios whose name contains X

WHAT A GREEN RUN MEANS, AND WHAT IT DOES NOT

This compiles the same gen_inhibit_core.c the ESP32 runs, so green means
"behaviour has not changed since the goldens were blessed". It is a
REGRESSION test. It cannot tell you a rule is right -- only that it is the
same. A wrong rule blessed into a golden stays green forever.

Two consequences worth keeping in mind:

  - --bless is not a way to make a failing test pass. Read the diff first and
    decide whether the CHANGE is intended. The per-scenario header says what
    the scenario is meant to demonstrate, which is the thing to check the new
    output against.
  - the goldens were blessed against code that has never run on the truck.
    Section 13 of the spec clears on BENCH proof, not on a green run here.
"""

import argparse
import difflib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCN = os.path.join(HERE, "scenarios")
GOLD = os.path.join(HERE, "golden")
RUNNER = os.path.join(HERE, "host_runner")


def sh(cmd, **kw):
    return subprocess.run(cmd, shell=isinstance(cmd, str), cwd=HERE,
                          capture_output=True, text=True, **kw)


def build():
    r = sh(["make", "-s", "host_runner", "extract_probe"])
    if r.returncode != 0:
        print("BUILD FAILED")
        print(r.stdout)
        print(r.stderr)
        sys.exit(2)
    if r.stderr.strip():
        print("build warnings:")
        print(r.stderr)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bless", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--context", type=int, default=3)
    args = ap.parse_args()

    build()
    if not os.path.isdir(SCN):
        print("no scenarios; run make_scenarios.py first")
        sys.exit(2)
    os.makedirs(GOLD, exist_ok=True)

    names = sorted(n[:-4] for n in os.listdir(SCN) if n.endswith(".scn"))
    if args.only:
        names = [n for n in names if args.only in n]
    if not names:
        print("no scenarios matched")
        sys.exit(2)

    npass = nfail = nnew = 0
    failed = []
    for name in names:
        with open(os.path.join(SCN, name + ".scn")) as fh:
            r = subprocess.run([RUNNER], stdin=fh, capture_output=True,
                               text=True, cwd=HERE)
        if r.returncode != 0:
            print("[ERROR ] %-30s runner exit %d" % (name, r.returncode))
            print(r.stderr[:2000])
            nfail += 1
            failed.append(name)
            continue

        out = r.stdout
        gpath = os.path.join(GOLD, name + ".trace")

        if name.startswith("replay-"):
            # A real-capture replay emits one TX line per received 0x051 --
            # tens of thousands of them, all identical in form. Diffing those
            # would put a 30,000-line golden in git to catch nothing the
            # summary does not: FINAL already carries tx_ok, tx_fail, ctr_ok,
            # ctr_bad and the histogram counts, so a change in transmit
            # behaviour still shows up. Keep the decision sequence, drop the
            # volume. TX payload CONTENT is pinned by the synthetic
            # b0-held-not-mirrored scenario instead, where it is readable.
            out = "".join(ln + "\n" for ln in out.splitlines()
                          if " TX " not in ln)

        if args.bless:
            with open(gpath, "w", newline="\n") as fh:
                fh.write(out)
            print("[BLESS ] %-30s %d lines" % (name, out.count("\n")))
            continue

        if not os.path.exists(gpath):
            print("[NEW   ] %-30s no golden -- run with --bless" % name)
            nnew += 1
            continue

        with open(gpath) as fh:
            want = fh.read()
        if want == out:
            print("[ok    ] %-30s %d lines" % (name, out.count("\n")))
            npass += 1
        else:
            print("[DIFF  ] %-30s" % name)
            d = difflib.unified_diff(want.splitlines(), out.splitlines(),
                                     "golden", "current", n=args.context,
                                     lineterm="")
            shown = 0
            for ln in d:
                print("    " + ln)
                shown += 1
                if shown > 60:
                    print("    ... (truncated)")
                    break
            nfail += 1
            failed.append(name)

    if args.bless:
        return
    print()
    print("%d passed, %d failed, %d without goldens" % (npass, nfail, nnew))
    if failed:
        print("failed: %s" % ", ".join(failed))
    sys.exit(1 if nfail else 0)


if __name__ == "__main__":
    main()
