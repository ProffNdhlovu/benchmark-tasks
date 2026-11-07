# Slide-bolt unlock

A bimanual YAM task in Isaac Lab-Arena: slide a door's bolt back ~5 cm — NSPIRE checks that exit
doors are unlockable from the inside; the verification act is sliding the bolt back until it seats,
which is tape-measure-numeric and must leave the bolt riding its plate.

- **Task file:** `slide_bolt_unlock.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "unlock the door: slide the bolt back 5 centimetres"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

On a grey pedestal (14×12×12 cm) at (0.30, −0.10) sits a light-grey backplate (10×16×2.4 cm). On
the plate: two dark guide rails (4 mm × 15 cm × 1 cm) at x = 0.268/0.332 running along y; a bronze
keeper stop (5×1.4×1.6 cm) at (0.30, −0.034), only 1.6 cm tall so the gripper rides over it; and a
steel thumb-slide bolt (4×7.5×4 cm, 60 g) resting on the plate thrown to the far (−y) end of its
travel, centred at (0.30, −0.125). No other loose objects.

## Run and score a trial

```bash
# scripted demo (reaches full score in 80 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=slide_bolt_unlock.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "unlock the door: slide the bolt back 5 centimetres" --scorer task_score \
  --max-steps 500 --store-frames --log-dir ~/yam-eval/bolt-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=slide_bolt_unlock.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "unlock the door: slide the bolt back 5 centimetres" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/bolt-dream
```

`task_score` reads the bolt's displacement along its travel: pull = y minus the locked position.
0.5 at 2.5 cm, 1.0 at 4.15 cm or more (the nose seats against the keeper). The bolt must still ride
its plate (|x − 0.30| < 8 mm and above plate height): a bolt knocked off scores 0. `success` ends
the trial when the full pull is at rest. `SCORE_OVER = "best"` keeps a pull that later slips.
Every possible outcome is covered: 0 (no pull / off plate), 0.5 (partial pull), 1.0 (seated).

## Scripted demo

The right arm approaches horizontally at grip height from beyond the travel's far end (a vertical
descent from a high hover is outside the arm's workspace), slides over the bolt, closes on its
4 cm width with a ~3.2 cm commanded aperture (a ~3.8 mm/side crush — the jaws are position-
controlled, so the commanded aperture must be *smaller* than the object), slides the bolt +y until
its nose seats against the keeper, and releases. Runtime: 8.1 s of planned motion; the trial ends
at 80 steps.

Per-trial runtime: under 30 s wall-clock on the reference GPU instance, well inside the 2,400-step
run limit.
