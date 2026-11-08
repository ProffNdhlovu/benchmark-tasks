# Two-handle faucet verification

A bimanual YAM task in Isaac Lab-Arena: turn both handles of a two-handle faucet fixture from fully
closed to fully open, without knocking a knob off its stem.

- **Task file:** `faucet_two_handle.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "turn both faucet handles to full open"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

Between the arms: a faucet body with a forward spout, two valve stems at y = ±6.5 cm, and a loose
square knob (5×5×4 cm, ~150 g) on each stem — blue (cold, left arm's side) and red (hot, right arm's
side). Four guide posts around each stem let a knob rotate but not slide off. Closed = knob aligned
with the fixture (0°); full open = 120° counter-clockwise, ±15°.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~25 s of wall-clock plan, 600-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=faucet_two_handle.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "turn both faucet handles to full open" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/faucet1-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=faucet_two_handle.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "turn both faucet handles to full open" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/faucet1-dream
```

`task_score` gives 0.5 per knob within 15° of full open and still seated on its stem, 1.0 for both;
`success_at_end` ends the trial once both hold for 1 s. Render videos with
`inspect-robots video <run.json>` (one MP4 per camera, written next to the run's frames).

## Runtime and seeds

- Scripted demo: ~25 s of simulated time (about 600 control steps at 20 Hz); a few minutes wall-clock
  including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`; the
  scripted demo is deterministic waypoints.
- Water is not simulated; "full open" is a handle-position check, readable from a photo — see
  `fidelity` in `task.json`.
