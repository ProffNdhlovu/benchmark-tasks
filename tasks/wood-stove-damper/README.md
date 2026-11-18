# Wood-stove flue damper

A wood stove sits on the station table with its flue opening framed by the top
plate, and the cast-iron slide-gate damper lies over the opening, covering it.
A damper left closed fills the room with smoke on the first fire of the season,
and the damper's position is invisible from outside the firebox - so the
operator slides the gate aside and judges the opening by eye. The robot
approaches the gate at face height from the stove's front, slides its open jaw
over the gate until the pads straddle the gate's two side faces, closes hard on
them, and carries the gate sideways along the stove's top until the flue
opening stands uncovered. The gate is a free rigid body throughout: closed it
bridges the opening, displaced it rests flat on the same surface with the flue
exposed.

Scoring: 1.0 when the gate, at rest on the stove's top, has moved at least
7.5 cm from its closed position (the flue uncovered); 0.5 for a 3.5-7.5 cm
move (the flue partly exposed); 0.0 closed, fallen to the table, or carried off
the stove's top. Do-nothing: 0 - the closed spawn is stable and cannot uncover
the flue on its own.

## Run and score a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=damper_slide.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "open the wood stove flue damper" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/damper-demo
```

`--scorer task_score` prints the trial's score; the same command with
`--policy dream --policy-checkpoint dreamlabs/dream-1-beta` runs the Dream-1
evaluation.

## Runtime and seeds

- The scripted policy completes the approach, the straddle, the close and the
  carry within the task's 1200-step run limit (~370 steps observed).
- `object_jitter=0` gives a deterministic scene; trials are reproducible
  without seeds.
