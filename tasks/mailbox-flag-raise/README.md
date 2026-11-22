# Raise a fallen mailbox flag

A rural-style mailbox with its signal flag lying flat in the channel on the box's top - the
down position, no outgoing mail. The robot grips the flag, lifts it so its far edge catches
the channel's stop rail, and the flag seesaws up over the rail to stand upright against it -
the outgoing-mail "up" position.

## Run and score a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mailbox_flag_raise.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "the mailbox flag is lying flat - raise it up" \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/flag-demo
```

The scorer reads the flag's final physical state: tilt from flat (deg), yaw, position on the
box, and rest condition. A model run uses `--policy dream --policy-checkpoint
dreamlabs/dream-1-beta` with the same `-E` flags and `--max-steps 2400`.

## Runtime per trial

The scripted demo completes in ~194 stored frames (~10 s of sim time at 20 Hz). The Dream-1
budget is 2,400 steps (2 minutes at 20 Hz).

## Seeds and determinism

`object_jitter=0` pins the starting layout; the task file places every object at a fixed
pose, so every trial starts identically.
