# Smoke alarm test — press TEST, let it spring back

A bimanual YAM task in Isaac Lab-Arena: press a smoke alarm's TEST button past its threshold and
let it spring back — the NSPIRE smoke-alarm check as a complete actuation sequence. A button held
down forever proves nothing; full credit requires the press AND the return to level.

- **Task file:** `smoke_alarm_test.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "test the smoke alarm: press its test button and let it spring back"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

A grey mounting pedestal (12×12×12 cm) at (0.30 m, −0.10 m) carries a white smoke-alarm disc
(r = 6 cm, 4.5 cm thick). On the alarm's top face a central fulcrum ridge (12×5×4 mm) carries a red
TEST bar (5×2.4×0.8 cm, ~30 g) resting level between two guide walls — free to tip about the ridge,
unable to slide off.

## Run and score a trial

```bash
# scripted demo (reaches full score in ~4 s of wall-clock plan, 400-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=smoke_alarm_test.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the smoke alarm: press its test button and let it spring back" \
  --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/smoke-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=smoke_alarm_test.py -E scene=station \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the smoke alarm: press its test button and let it spring back" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/smoke-dream
```

`task_score`: **1.0** when the TEST bar was pressed past its threshold (either end dipping 3 mm,
about 14°) and has returned to within 2 mm of level; **0.5** when it was pressed but is still held
down; **0.0** when it was never pressed. `success` ends the trial once the press has latched and the
bar is back level, held 0.4 s; `SCORE_OVER = "end"`.

## Runtime and seeds

- Scripted demo: ~4 s of simulated time (about 80 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- The alarm's sounder is not simulated: the latched press plus the level return stands in for
  "the alarm sounds". The TEST button is a gravity-restoring rocker — a free push-button cannot
  spring back in this simulator, and the rocker reproduces the same externally visible evidence.
  See `fidelity` in `task.json`.
