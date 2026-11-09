# Toilet flush test with a filed report

A YAM task in Isaac Lab-Arena: a table-top toilet with a tank-mounted flush lever - a small rocker
bar on a fulcrum ridge between guide walls. Press the lever's front end down past its tilt threshold
and let it spring back, then file the report by leaving a white tag on the matching pad: green (the
flush works), yellow (the lever is seized), red (the handle is missing).

- **Task file:** `toilet_flush_test.py` (fixture, three conditions, report pads, scoring, scripted
  demo — one file)
- **Instruction:** "test the toilet flush and file the report"
- **Scene:** `station` (Dream's calibrated YAM station)
- **Per-trial condition:** the `FAULT_MODE` environment variable — `ok`, `stiff` (the lever is a
  fixed, seized bar) or `missing` (no lever; only the bare pivot mount). The condition is not
  announced to the policy.

## Run and score a trial

```bash
# scripted demo (reaches full score in every mode; 900-step cap)
FAULT_MODE=stiff inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=toilet_flush_test.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the toilet flush and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/toilet-demo

# Dream-1 run (900 steps = the task's run limit)
FAULT_MODE=ok inspect-robots run --embodiment yam_arena --policy dream \
  -E task=toilet_flush_test.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the toilet flush and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/toilet-dream
```

`task_score`: 1.0 for the tag on the pad matching the lever's actual condition, 0.3 on a wrong pad,
0.0 elsewhere. Run all three modes for a full picture (`FAULT_MODE=ok`, `stiff`, `missing`).

## Runtime and seeds

- Scripted demo: ~15 s of simulated time (about 300 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 900 control steps (45 seconds at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout for its mode; the
  scripted demo is deterministic waypoints, and the mode it plays is the mode being scored.

## Why this task

The flush is the one functional test a toilet gets, and NSPIRE deficiency D181 covers exactly the
three findings this task encodes: a component that is damaged, inoperable, or missing such that it
may limit the resident's ability to safely discharge human waste. The diagnosis must be made by
manipulation (press the handle, feel the travel) and committed as a physical report; guessing
scores about 1/3 in expectation, so the report discriminates real diagnosis.
