# Solar panel tilt-up

A small solar panel is stored flat on its own mounting stand on the station table: its far end
rests on a storage post, its near end rests on the stand's crossbar — the way crews store panels
flat on their mounting frames between seasons. A panel left flat sheds most of its winter yield,
so re-angling it onto the stand's 30-degree operating tilt is routine seasonal work. The robot
grips the panel's two faces and rolls it up: the panel pitches about the crossbar's front edge
inside the pinch until its bottom edge seats at the toe rail, leaning back at the stand's tilt.
Single fixed condition (the flat-stored panel is the layout).

Scoring: 1.0 when the panel rests leaning back on the crossbar at 25-35 degrees from horizontal
with its bottom edge seated within 3 cm of the toe lip's face; 0.5 for a 35-42 degree lean
(overshot the seat); 0.0 flat, overturned, fallen, facing away, or propped anywhere else.
Do-nothing: 0 — the spawn is a stable hinge-line seat, so the panel cannot fall into the scoring
pose on its own.

## Run and score a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=panel_tilt_stand.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "tilt the solar panel up onto its stand" \
  --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/panel-demo
```

`--scorer task_score` prints the trial's score; the same command with
`--policy dream --policy-checkpoint dreamlabs/dream-1-beta` runs the Dream-1 evaluation.

## Runtime and seeds

- The scripted policy completes the descent, the close and the roll-up within the task's 400-step
  run limit (~330 steps observed; the policy's waypoint clock maps ~100 runner steps per second).
  The roll-up itself happens during the jaw's close: the pads' squeeze wedges the panel's near end
  over the crossbar's front edge and it settles at ~29 degrees in under a second.
- `object_jitter=0` gives a deterministic scene; trials are reproducible without seeds.
