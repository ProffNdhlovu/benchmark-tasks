# Pull-chain light fix

A pull-chain light fixture stands on a post: an empty twist-lock socket on top, and beside it a
pull rod balanced level across a pivot ridge. The bulb has fallen out of its socket and stands on
the table. The inspector pulls the rod's grip end down (closing the switch), seats the bulb back
into the socket, and files the finding. Single fixed condition (the fallen
bulb is the layout).

Scoring: 0.5 when the rod was tilted past the pull threshold (the switch closed); +0.5 when the
bulb is seated back in its socket and the tag is filed on the green pad (or the bulb still off its
seat with the tag on the yellow pad). 1.0 for the complete fix-and-test. Do-nothing: 0.

## Run and score a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pullchain_light_fix.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fix the pull-chain light: seat the bulb and test it" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/bulb-demo
```

`--scorer task_score` prints the trial's score; the same command with
`--policy dream --policy-checkpoint dreamlabs/dream-1-beta` runs the Dream-1 evaluation.

## Runtime and seeds

- The scripted policy completes the pull, the bulb reseat and the filing in ~45 s (900 steps at
  20 Hz), within the task's 2,400-step run limit.
- `object_jitter=0` gives a deterministic scene; trials are reproducible without seeds.
