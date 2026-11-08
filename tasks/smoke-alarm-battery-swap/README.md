# Smoke alarm battery swap — pull the dead cell, insert the fresh one, reseat

A bimanual YAM task in Isaac Lab-Arena: lift a smoke alarm off its mount, pull the dead battery,
insert the fresh one, and reseat the alarm — the NSPIRE smoke-alarm battery fix as a complete
maintenance sequence. Four grasps, one placement with centimetre tolerance, and a reseat that must
land level.

- **Task file:** `smoke_alarm_battery_swap.py` (scene objects, `success()`, `score()`, scripted
  demo — one file)
- **Instruction:** "swap the smoke alarm's dead battery for the fresh one and reseat the alarm"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

A grey mounting frame (four 1-cm borders, 20×20 cm outer / 18×18 cm inner opening) lies flat on the
table centred at (0.35 m, −0.10 m) so everything inside rests on the table. A white square smoke
alarm (5.2×5.2×6.5 cm) sits at the frame's centre. The dead battery (4.4×4.4×1 cm) lies in the tray
spot beside it at (0.38 m, −0.05 m); the fresh battery lies in the tray spot at (0.31 m, −0.04 m).

## Run and score a trial

```bash
# scripted demo (reaches full score in ~40 s of wall-clock plan, 1,000-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=smoke_alarm_battery_swap.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "swap the smoke alarm's dead battery for the fresh one and reseat the alarm" \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/battery-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=smoke_alarm_battery_swap.py -E scene=station \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "swap the smoke alarm's dead battery for the fresh one and reseat the alarm" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/battery-dream
```

`task_score`: **0.5** when the dead battery rests on the table at least 6 cm from the frame's
centre; **+0.5** when the fresh battery rests in the tray spot (within 2 cm) with the alarm
reseated level on its seat (within 2.5 cm, within 15°). **1.0** for the full swap. Do-nothing
scores 0. `SCORE_OVER = "best"`.

## Runtime and seeds

- Scripted demo: ~816 control steps at 20 Hz (about 41 s of simulated time); a few minutes
  wall-clock including Isaac Sim warmup. Verified `task_score: 1.0` on three consecutive runs.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- The alarm's sounder and twist-to-release bayonet are not modelled; the compartment is a tray spot
  beside the alarm (see `fidelity` in `task.json`).
