# Window sash test (NSPIRE D203 — window will not stay open)

A light window sash stands on an 8 cm sill block in front of a low backdrop board. The robot
grips the sash, raises it, lets go, and watches: in `stays` the sash settles back onto the block
and stays up (the balance holds); in `falls` the block is absent, the sash stands on the table,
and it slides back down (the balance is broken). The robot then files the white report tag on the
matching pad — green for *stays open*, yellow for *falls shut*.

## Run a trial

Per-trial condition via the `FAULT_MODE` environment variable:

```bash
FAULT_MODE=stays inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=window_sash_test.py -E scene=station -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "lift the window sash, release it and file the report" \
  --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/window-demo-stays

FAULT_MODE=falls inspect-robots run ... # same flags, --log-dir ~/yam-eval/window-demo-falls
```

## Scoring

`task_score`: 1.0 when the white tag ends on the pad matching the sash's actual condition
(tag centre within 2.5 cm of the pad centre at the pad-top height), 0.3 on the wrong pad,
0.0 otherwise. Read from the tag's final resting spot — a photo of the end state is enough.

## Runtime and seeds

~25 s of sim per trial (the scripted plan completes around step 500 of the 900-step / 45 s
limit). Deterministic: `object_jitter=0`, no randomization, no seeds. Both conditions verified:
`stays` → 1.0 (the sash returns to the block, tag on green), `falls` → 1.0 (the sash returns to
the table, tag on yellow).
