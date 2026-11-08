# Faucet function test with a filed report

A bimanual YAM task in Isaac Lab-Arena: test a two-handle faucet by manipulating its handles, decide
whether it works, will not open, or will not shut off, and file the report by leaving a white tag on
the matching pad — green (works), yellow (will not open), purple (will not shut off).

- **Task file:** `faucet_function_report.py` (fixture, three fault conditions, report pads, scoring,
  scripted demo — one file)
- **Instruction:** "test the faucet and file the report: leave the tag on the pad matching what you
  find"
- **Scene:** `station` (Dream's calibrated YAM station)
- **Per-trial condition:** the `FAULT_MODE` environment variable — `ok`, `wont_open` or
  `wont_shut_off`. The condition is not announced to the policy; the fault, when present, is a
  physical worn-stop post that blocks the cold handle's travel (the hot handle always works).

## Run and score a trial

```bash
# scripted demo (reaches full score in every mode; ~30 s of plan, 700-step cap)
FAULT_MODE=wont_open inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=faucet_function_report.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the faucet and file the report" --scorer task_score \
  --max-steps 700 --store-frames --log-dir ~/yam-eval/fault-open

# Dream-1 run (2,400 steps = the task's run limit)
FAULT_MODE=wont_open inspect-robots run --embodiment yam_arena --policy dream \
  -E task=faucet_function_report.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the faucet and file the report" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/fault-open-dream
```

`task_score`: 1.0 for the tag on the pad matching the fixture's actual condition, 0.3 on a wrong pad,
0.0 elsewhere; `success_at_end` ends the trial once the tag rests on the correct pad for 1 s. Run all
three modes for a full picture (`FAULT_MODE=ok`, `wont_open`, `wont_shut_off`). Render videos with
`inspect-robots video <run.json>`.

## Runtime and seeds

- Scripted demo: ~30 s of simulated time (about 600 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout for its mode; the
  scripted demo is deterministic waypoints, and the mode it plays is the mode being scored.

## Why this task

The report — not the manipulation — is the deliverable, mirroring how inspections work: the inspector
must distinguish a fixture fault from a fumbled attempt and commit to a finding. Guessing scores 1/3
in expectation; a policy has to actually test the handle to file correctly.
