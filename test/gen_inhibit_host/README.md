# gen_inhibit host harness

Runs `main/gen_inhibit_core.c` — the same file the ESP32 runs — on a PC, with
no hardware, and diffs its behaviour against recorded goldens.

Written 2026-09-19, when the decision logic was split out of `gen_inhibit.c`
into a pure core plus a driver shim.

## Read this before trusting a green run

**This is a regression harness, not a differential one.** It compiles the same
core the target runs, so green means *behaviour has not changed since the
goldens were blessed*. It cannot catch a rule that is wrong here and wrong in
the golden too.

That trade was made deliberately with the user on 2026-09-19. The alternative
— maintaining an independent Python reference and diffing two implementations,
the way `projects/vtrux/tools/interposer/firmware/test/host_diff/` does with
`machine.py` against `machine.cpp` — was considered and declined. We have no
1:1 reference for this logic: `tools/gen_inhibit/inhibit.py` is proactive and
scheduled where this is a reactive trail, so it is related, not an oracle.

Why it is still worth having: every bug found in this component so far has
been a **state-sequence** bug, reachable only from a particular order of
received frames and invisible to a compiler. That is exactly what a replay
catches. Three were found by hand on 2026-09-19; two more were found by this
harness on its first run (see *Findings*).

**`--bless` is not a way to make a failing test pass.** Read the diff, decide
whether the change is intended, and check it against what the scenario's own
header says it is meant to demonstrate. A wrong rule blessed into a golden
stays green forever.

**A green run does not clear a spec §13 item.** Those clear on bench proof.

## Layout

| File | What it is |
|---|---|
| `host_runner.c` | Emulates the worker loop, replays a scenario, prints a deterministic trace |
| `extract_probe.c` | Exposes the core's four hand-rolled extractors for the cantools cross-check |
| `Makefile` | `make` builds both; `make asan` rebuilds under ASan/UBSan |
| `make_scenarios.py` | Generates the synthetic scenarios into `scenarios/` |
| `from_capture.py` | Turns a real capture into a scenario; `--scan` finds key-on points |
| `run_tests.py` | Builds, replays every scenario, diffs against `golden/` |
| `test_signals.py` | 2000 randomised frames per signal, C extractors vs cantools |
| `scenarios/` | Generated inputs. Not hand-edited |
| `golden/` | Blessed traces |

## Running it

```sh
make
python3 make_scenarios.py      # writes scenarios/
python3 run_tests.py           # build, replay, diff
python3 run_tests.py --only keyon
python3 run_tests.py --bless   # after reading the diff
python3 test_signals.py        # needs cantools and the project repo
```

`test_signals.py` and `from_capture.py` reach into the **project** repo
(`reverse-it`), which is a separate tree — for the DBCs and for `canre`'s
parsers. Point them at it with `--repo` or `$GEN_INHIBIT_REPO`; the default is
the `madhouse-debian` layout, `~/Seafile/NotGit/reverse-it`.

Prefer running all of this on `madhouse-debian`: the log corpus is on local
disk there, so no SeaDrive hydration is involved, and it keeps the load off
whichever machine is driving the bench.

## What is emulated, and what is therefore untested

`host_runner` reproduces the worker loop closely enough for frame ordering and
for the interaction between the periodic tick and frame arrival — which is
where the bugs live. It does **not** emulate:

- the TWAI driver, `can_enable()`/`can_disable()`, or the listen-only forcing
  for OBSERVE. Those are in the shim and are hardware-only;
- **latency**. Every transmit is instantaneous, so the response histogram is
  all zeros. Timing is what the bench ESP-to-ESP test measures; a host replay
  cannot speak to it;
- preemption, and the races between the worker task and the HTTP handlers.

## Findings

Both were produced by this harness on its first run. Both are **pre-refactor
behaviour, faithfully preserved** — neither is damage from the core split —
and both are recorded as goldens rather than fixed, because the spec is the
source of truth and a behaviour change belongs there first.

**1. The arm gate's "generator already running" check is arrival-order
dependent.** `arm-gate-order-rpm-first` and `arm-gate-order-cmd-first` differ
by one microsecond in whether 0x051 or 0x054 lands first. Rpm-first blocks
correctly and never transmits. Cmd-first finds `have_rpm` still false, skips
the check, goes live, and transmits ~16 zero-torque frames at a generator
turning 800 rpm before the 0.3 s runtime debounce ends the run. The check
exists because "taking over a loaded generator and commanding zero sheds the
engine's whole load in one frame".

**Severity is lower than that reads, measured after the fact.** In 21,925 real
frames with the generator at >=300 rpm, `gen_rpm_ref` was never the engine-off
null and torque was never zero — so the gate's two `0x051`-based checks, which
can never be skipped, already cover the loaded case. The synthesised
combination occurs 8 times in 39,718 co-observed frames across 150 logs, over
a 73 ms engine coast-down where the VCM has already commanded zero, and going
live there is correct. Real gap, no observed hazard.

**2. A latched section 6 disable freezes `inhibit_live` at true.**
`disable-freezes-live-flag`. Transmission stops correctly, but the interlock
block is guarded by `mode == INHIBIT && !disabled`, so once disabled nothing
clears the flag and the worker never reaches its OFF branch. `diag_flags` bit5
and the JSON then claim a live inhibit on a device that is latched off. Same
defect class as the disarm bug fixed by hand earlier the same day.

## Regenerating the real-capture replays

`scenarios/` is gitignored — the replays are 25k–127k lines each. The goldens
are committed (summary only; TX lines are filtered, since `FINAL` already
carries `tx_ok`/`tx_fail`/`ctr_ok`/`ctr_bad`). To rebuild the inputs:

All five captures are copied into the project repo so this suite has one
stable source instead of reaching into the Android auto-capture store. They
are described in `projects/vtrux/logs/README.md` there. The copy is not
ceremony: the original `replay-T20-drive` fixture broke precisely because its
source moved out from under the suite.

```sh
L=~/Seafile/NotGit/reverse-it/projects/vtrux/logs
python3 from_capture.py $L/vtrux_20260719_190019_T4.log  --at 0 --for 220 \
        --out scenarios/replay-genrun-stop.scn
python3 from_capture.py $L/vtrux_20260323_220148_T0.log  --at 0 --for 300 \
        --out scenarios/replay-healthy-engine-off.scn
python3 from_capture.py $L/vtrux_20260322_165622_T1.log  --at 0 --for 64 \
        --out scenarios/replay-mmode-genstart.scn
python3 from_capture.py $L/vtrux_20260714_112312_T2.log  --at 0 --for 225 \
        --out scenarios/replay-shutdown-at-keyon.scn
python3 from_capture.py $L/vtrux_20260802_123318_T0.log  --at 0 --for 297 \
        --out scenarios/replay-bus-sleeps.scn
```

`--scan` finds key-on candidates in a capture you want to add. Two
corpus-wide selectors live in
`projects/vtrux/notes/artifacts/gen-inhibit/`: `shutdown_and_wake_scan.py`
found `replay-shutdown-at-keyon` and `replay-bus-sleeps`, and
`replay_candidate_scan.py` found `replay-genrun-stop`.

**Check a candidate's provenance before you adopt it.** A capture recorded
with test equipment inline on the bus looks exactly like an ordinary drive,
and one such capture was in this suite for a day before it was caught. The
known rig dates are listed in the repo's `data-sources.md`.

## Real-capture replays

Five captures, five different correct outcomes.

**`replay-genrun-stop`** — 220 s, 93,150 frames, SoC 77.2 %, and the only
replay that shows the arm gate both holding and releasing:

| t | event | device |
|---|---|---|
| 0.007 s | generator already turning at 735 rpm | blocked, `generator running` |
| 0.007–92.058 s | generator runs, 1,676 rpm peak | **transmits nothing** |
| **92.058 s** | generator stops | goes live in the same tick |
| 92.058–220.476 s | engine off, SoC 77 % | 12,779 transmits, 0 failures |
| 220.476 s | capture ends | bus-loss abort on the trailing silence |

21,982 counter steals, `ctr_bad=0`. This is the fixture that says the gate is
not one-way: it holds the device off for 92 s of real generator operation and
then arms itself on the real stop, with no synthetic directive anywhere in
the trace but the initial mode set.

**It replaces `replay-T20-drive`, removed 2026-09-20.** That fixture's source
was `vtrux_20260617_193900_T20`, from the session where the `generator_runner`
laptop bridge was inline on the bus; the user ruled those captures out as test
data. Its role was *generator running plus SoC below the 21 % floor*, and
**that combination does not exist anywhere else in the corpus** — all 26
captures with both properties are from the 2026-06-17/18 rig window
(`projects/vtrux/notes/artifacts/gen-inhibit/replay_candidate_scan.py`). So
the replacement keeps the generator-running block against real traffic and
gives up the low-SoC latch, which `low-soc-debounce` already asserts
synthetically. The low-SoC latch is no longer exercised against real traffic.

**`replay-healthy-engine-off`** — 300 s, 127,082 frames, engine off at SoC
84.6 %. Live at 7 ms, **held the whole capture**, 29,990 transmits, zero
aborts. This is the one that says the §7 trip set is not hair-triggered
against real traffic.

**`replay-mmode-genstart`** — 64 s, 24,809 frames, and the most complete
end-to-end story in the corpus. The driver engages M mode and the generator
actually starts:

| t | event | device |
|---|---|---|
| 0.006 s | engine off, SoC 43.3 % | goes live |
| 0.006–39.057 s | normal driving, shifter P/R/N/D | 3,905 transmits |
| **39.057 s** | `shift_lever_pos` -> 4 (**M mode**) | latches `m_mode`, `inhibit_live` clears, transmission stops |
| 46.4 s | **generator cranks and runs** | silent |
| 53.2 s | M mode released | latch **holds** |
| 54.4 s | engine coasts down | silent |

**Zero transmits after the latch** — last at 39.054 s, latch at 39.057 s.
This exercises §6.1's entire purpose against a real event rather than a
synthetic one: the driver demanded the generator, the inhibitor stood down,
and the generator started. SoC rises 43.3 % -> 54.9 % across the capture,
which is the generator doing its job with the inhibitor out of the way.

Across the replays the rolling counter stepped exactly +1 on every
transition: `ctr_bad=0` in all five goldens.
