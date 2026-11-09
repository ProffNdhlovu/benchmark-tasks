# Grab bar secure test (NSPIRE D090)

Grip the grab bar, pull it along its axis, and file the report tag on the pad matching the finding:
green = the bar holds (secure condition), yellow = the bar pulls out of its rails (loose condition).
Score 1.0 for a correct report, 0.3 for a report filed on the wrong pad, 0.0 otherwise.

## Run a trial

Source the arena environment, then:

```bash
FAULT_MODE=secure inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=grab_bar_test.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the grab bar and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/grabbar-demo
```

`FAULT_MODE` seeds the condition: `secure` (raised end stops capture the bar) or `loose`
(stops absent, the bar slides out of its rails under a pull).

Score a trial from the run's `adhoc_*.json`: `results.metrics` reports the final `task_score`
(1.0 correct report / 0.3 wrong pad / 0.0 no report). The scripted demo reaches 1.0 in both
conditions; the bar's displacement in the demo log shows the pull's ground truth.

## Runtime

A 900-step scripted demo runs in ~3-4 minutes wall time on one L40S-class GPU (or one GB10),
including Isaac Sim boot. The Dream-1 model run is capped at 2,400 steps (2 minutes of
control at 20 Hz), ~10-15 minutes wall including boot and service retries.

## Seeds

`object_jitter=0` pins the starting layout; the trial condition is set solely by `FAULT_MODE`.
