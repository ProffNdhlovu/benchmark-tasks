# Cooktop knob sweep — find the knob left on

A bimanual YAM task in Isaac Lab-Arena: one of a four-knob cooktop's burner knobs starts a
quarter-turn open; the robot must scan the controls, find the wrong knob, and turn it off — the
NSPIRE cooking-appliance check. No blind rotation solves it: a single fixed sweep leaves either the
on knob mid-way or an off knob opened, so the search is real.

- **Task file:** `cooktop_knob_sweep.py` (scene objects, `success()`, `score()`, scripted demo — one file)
- **Instruction:** "one of the cooktop's burner knobs is turned on; find it and turn it off"
- **Scene:** `station` (Dream's calibrated YAM station)

## Fixture

A dark cooktop slab (35×22.5×2 cm) centred at (0.305 m, −0.0175 m) with four burner rings (r = 4 cm)
and a raised control console (35×6×6 cm) along its front edge. Four square knobs (5×5×4 cm, 150 g)
on steel stems (r = 1.4 cm, 3 cm) at x = 0.17, 0.26, 0.35, 0.44 m, y = 0.065 m, each riding on four
small guide posts. Knob 2 (x = 0.26 m) starts a quarter-turn open (90°); the rest start closed.

## Run and score a trial

```bash
# scripted demo (reaches full score, ~18 s of wall-clock plan, 600-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=cooktop_knob_sweep.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "one of the cooktop's burner knobs is turned on; find it and turn it off" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cooktop-demo

# Dream-1 run (2,400 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=cooktop_knob_sweep.py -E scene=station \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "one of the cooktop's burner knobs is turned on; find it and turn it off" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/cooktop-dream
```

`task_score`: **1.0** when every knob sits within 15° of a closed orientation (0/180° — a square
knob is 180-deg symmetric); **0.5** when the on knob was shut but an off knob was left opened by
collateral turning; **0.0** when the on knob is still open. Do-nothing scores 0. `success` ends the
trial when all four are closed and it holds 0.4 s; `SCORE_OVER = "end"`.

## Runtime and seeds

- Scripted demo: ~18 s of simulated time (about 360 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 2,400 control steps (2 minutes at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout in `setup.png`;
  the scripted demo is deterministic waypoints.
- No gas or flame is simulated: 'on' is the knob's angular position and 'off' its return to a
  closed orientation — what a photo of the controls reads. The knob's yaw transfer from the wrist
  is friction-dependent, so the model must grip and turn deliberately. See `fidelity` in `task.json`.
