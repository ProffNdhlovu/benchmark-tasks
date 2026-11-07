# Thermostat to 120

A bimanual YAM task in Isaac Lab-Arena: turn a water heater's thermostat dial from the 140 mark
down to 120 — water heaters set above 120 deg F scald, and the dial's pointer against its scale is
the numeric evidence an inspection reads.

- **Task file:** `thermostat_120.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "turn the water heater thermostat down to 120 degrees"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

At (0.30, 0.065): a dark control box (14×10×8 cm) carries a white dial face (12×8.5×1 cm) on a grey
stem (2.8 cm across, 3 cm tall). Four black posts (1.2 cm across, 4.2 cm tall) at ±3.11 cm in x and
y guide the dial. An orange dial knob (5×5×4 cm, 150 g) rides the stem, its pointer at the 140 mark
(60° clockwise of the 120 mark); the face's scale is 1° of pointer per deg F. No other loose
objects.

## Run and score a trial

```bash
# scripted demo (reaches full score in 296 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=thermostat_120.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "turn the water heater thermostat down to 120 degrees" --scorer task_score \
  --max-steps 500 --store-frames --log-dir ~/yam-eval/tstat-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=thermostat_120.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "turn the water heater thermostat down to 120 degrees" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/tstat-dream
```

`task_score` reads the dial's yaw about the stem: 1.0 within 7.5° of the 120 mark (±5 deg F on the
face), 0.5 within 15°, else 0. The dial must stay on its stem — the posts keep it from lifting, and
a dial knocked off is not a set thermostat. `success` ends the trial once the pointer holds inside
the full tolerance. `SCORE_OVER = "end"` scores the final state. Every possible outcome is covered:
0 (untouched or knocked off), 0.5 (partway), 1.0 (at 120).

## Scripted demo

The left arm uses the proven knob recipe: a horizontal approach at grasp height, a slow descent, a
close on the dial's 5 cm width with a ~4.6 cm commanded aperture, then twelve 1 s incremental
waypoints sweeping the wrist −72° (the measured transfer is ~0.83, so 60° of dial per 72° of
wrist), then a release and retreat. The right arm parks. Runtime: ~15.4 s of planned motion; the
trial ends at 296 steps.

Per-trial runtime: under 30 s wall-clock on the reference GPU instance, well inside the 2,400-step
run limit.
