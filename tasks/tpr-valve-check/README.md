# TPR relief valve test with a filed report

A YAM task in Isaac Lab-Arena: a table-top water heater carries a temperature-pressure relief (TPR)
valve on its top with a square test knob on a stem, held by four guide posts. Turn the test knob a
quarter-turn to its test position - on a working valve it swings through to 90 degrees and stays; on
an obstructed one a corrosion bump stops it at about 25 degrees - then file the report by leaving a
white tag on the matching pad: green (valve operates) or yellow (control will not swing through).

- **Task file:** `tpr_valve_check.py` (fixture, two conditions, report pads, scoring, scripted
  demo — one file)
- **Instruction:** "test the TPR relief valve and file the report"
- **Scene:** `station` (Dream's calibrated YAM station)
- **Per-trial condition:** the `FAULT_MODE` environment variable — `ok` (knob swings through) or
  `blocked` (a corrosion bump stops the knob at ~25 deg). The condition is not announced to the
  policy; the fault is a physical bump in the knob's swept circle.

## Run and score a trial

```bash
# scripted demo (reaches full score in both modes; 900-step cap)
FAULT_MODE=blocked inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tpr_valve_check.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the TPR relief valve and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tpr-demo

# Dream-1 run (900 steps = the task's run limit)
FAULT_MODE=ok inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tpr_valve_check.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "test the TPR relief valve and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/tpr-dream
```

`task_score`: 0.5 for the knob turned at least 15 deg off closed and still seated on its stem,
+0.5 for the tag on the pad matching the fixture's actual condition, 1.0 for the complete check,
0.0 for do-nothing. Run both modes for a full picture (`FAULT_MODE=ok`, `blocked`).

## Runtime and seeds

- Scripted demo: ~20 s of simulated time (about 400 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 900 control steps (45 seconds at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout for its mode; the
  scripted demo is deterministic waypoints, and the mode it plays is the mode being scored.

## Why this task

The TPR valve is a water heater's last defense against a scald or a tank explosion, and NSPIRE
deficiency D198 covers an obstructed valve — the inspector works the test control through its
travel. The finding must be filed physically (a tag on the matching pad), so a policy cannot score
by photographing the scene; guessing the condition scores 0.5 in expectation. The knob-turn reuses
the proven square-knob mechanism (guide posts block translation, the pads turn the knob on its
flats), and the fault is a physical bump an inspector's hand would feel.
