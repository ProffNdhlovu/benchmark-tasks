# Fit an adjustable cane to the height marked on the bench

A mobility-aid fitting task on the yam_arena station: an adjustable square-tube cane lies on two cradle rails
next to a steel ruler, with the client's fitted height (80.0 cm) marked as a red line. The agent pulls the lower
shaft out until it stops with the rubber foot on the red line, shoves the shaft in once toward the foot and works
it back out to the stop to check the button holds, then lets go.

The mechanism is physical: the lower shaft rides a prismatic joint in the cane's articulation whose upper limit
sits at the marked hole, so once the shaft is pulled out the button's shoulder seats against the hole's edge and
the shaft can come no further out; along the travel it stays where it is left. The fitter pulls the shaft out to
the stop (the foot on the red line), shoves it in once, works it back out to the stop, and lets go.

## Files

- `cane_height_fit.py` — the task: scene objects (`cane` mechanism, `cradle`, `ruler`), `score`, `success`,
  `distance`, and the scripted `demo`.
- `yamkit.py`, `guardkit.py` — modeled-prop / mechanism and gripper-contact helpers, taken from the
  `oil-dipstick-wipe-reinsert-read` task in this repository (unchanged).
- `task.json` — the submission record.

## Run and score a trial

```
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=<path>/cane_height_fit.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the clients cane on the bench: pull the lower shaft out until it stops with the rubber foot on the red line, shove it in once and work it back out to the stop, then let go." \
  --scorer task_score --max-steps 800 --store-frames --log-dir <out>
```

For a model run, swap `--policy yam_scripted` for the policy under test and raise `--max-steps` to 2400
(2 minutes at 20 Hz). The scorer prints `task_score`; `success` ends the trial when all three components hold
together for 1 s.

## Timing and seeds

The scripted demo is ~15 s of arm motion (~295 control steps at 20 Hz) and reaches 1.0. With
`-E object_jitter=0` every trial starts from the exact layout in `setup.png`; the scene has no other random
elements, so seed 0 is the recorded trial.

## Scoring detail

- **0.5 fit** — the shaft left at its end of travel (25.0 cm of extension, within 4 mm), foot on the red line.
  Any position short of the stop scores 0.
- **0.3 hold test** — after the fit was first reached, the shaft was shoved at least 6 mm in from the stop at
  some step and is back within 4 mm of the stop at the end.
- **0.2 let-go** — at the end no gripper touches the cane (palm and fingers checked against their own shape)
  and the shaft is still (under 1 cm/s).
