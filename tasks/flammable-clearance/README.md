# Flammable clearance: move the box off the space heater

A YAM task in Isaac Lab-Arena: a table-top space heater has a cardboard box sitting on top of it - a
flammable item stored on a heat appliance (NSPIRE D078, life-threatening, 24-hour correction). A
heavy metal pot on the table beside it is the non-flammable decoy. The robot lifts the box off the
heater, carries it across the table and sets it down in the green marked safe zone.

- **Task file:** `flammable_clearance.py` (heater, box, decoy pot, safe zone, scoring, scripted
  demo — one file)
- **Instruction:** "move the flammable box off the space heater to the safe zone"
- **Scene:** `station` (Dream's calibrated YAM station)
- **Single condition** - no `FAULT_MODE`. The discrimination the policy has to make is flammable
  cardboard vs the heavy metal pot that must be left alone.

## Run and score a trial

```bash
# scripted demo (reaches full score; 900-step cap)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=flammable_clearance.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "move the flammable box off the space heater to the safe zone" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flamm-demo

# Dream-1 run (900 steps = the task's run limit)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flammable_clearance.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "move the flammable box off the space heater to the safe zone" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/flamm-dream
```

`task_score`: 0.5 for the box at rest and off the heater (centre ≥9 cm from the heater's centre),
+0.5 for the box at rest in the safe zone (centre within 3 cm of the zone's centre), 1.0 for the
complete clearance, 0.0 for do-nothing.

## Runtime and seeds

- Scripted demo: ~12 s of simulated time (about 230 control steps at 20 Hz); a few minutes
  wall-clock including Isaac Sim warmup.
- Dream-1: up to 900 control steps (45 seconds at 20 Hz).
- No seed dependence: `-E object_jitter=0` pins every object to the exact layout; the scripted demo
  is deterministic waypoints.

## Why this task

A flammable item on a heat appliance is a life-threatening NSPIRE finding with a 24-hour correction
window - the highest-stakes routine check a housing robot would run. The clearance is a real
manipulation with a judgment call (which item is the hazard), scored numerically from object
positions: a photo alone cannot score it, and moving the wrong object gains nothing.
