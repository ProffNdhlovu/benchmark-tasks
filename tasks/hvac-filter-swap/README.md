# HVAC filter swap — pull the dirty cartridge, seat the clean one flush

A bimanual YAM task in Isaac Lab-Arena: pull a clogged return-grille cartridge out of its slot and
seat a clean one flush — the NSPIRE ventilation check as a slide-swap. The dirty cartridge is
pulled through the mouth, set down clear of the insert path; the clean one is carried to the mouth
and pushed in until the back stop seats it flush.

- **Task file:** `hvac_filter_swap.py` (scene objects, `success()`, `score()`, scripted demo —
  one file)
- **Instruction:** "swap the dirty air filter for the clean one and seat it flush"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

A grey grille frame at x = 0.36 m: two jambs (1×5.5×6.5 cm each, 7.5 cm apart inside), a sill and a
back stop form a square slot 5.5 cm deep, open above, mouth facing −y. The dirty cartridge
(5.2×5.2×6 cm, tan) sits in the slot on the sill; the clean one (white) lies on the table at
(0.43 m, −0.14 m).

## Run and score a trial

```bash
# scripted demo (reaches full score in ~480 control steps, 900-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hvac_filter_swap.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "swap the dirty air filter for the clean one and seat it flush" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/hvac-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hvac_filter_swap.py -E scene=station \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "swap the dirty air filter for the clean one and seat it flush" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hvac-dream
```

`task_score`: **0.5** when the dirty cartridge rests on the table at least 2.6 cm beyond the
frame's mouth plane (its centre past its half-depth), within 20 cm of the frame's x; **+0.5** when
the clean cartridge rests in the slot within 1.2 cm of the seated position on x, y and z. **1.0**
for both. Do-nothing scores 0. `SCORE_OVER = "best"`.

## Runtime and seeds

- Scripted demo: ~480 control steps at 20 Hz (about 24 s of simulated time); a few minutes
  wall-clock including Isaac Sim warmup. Verified `task_score: 1.0` on three consecutive runs.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- The filter is a solid cassette whose colour stands in for dust loading; the slot seats on a stop
  exactly as a slide-in cartridge does (see `fidelity` in `task.json`).
