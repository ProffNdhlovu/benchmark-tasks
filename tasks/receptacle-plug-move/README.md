# Move a lamp's plug from a scorched socket to the sound one

Two receptacles stand side by side. The left one's faceplate is visibly scorched - a burned
contact that can arc - and a lamp's plug is seated in it. The robot pulls the plug out of the
scorched socket and seats it in the sound one beside it. Scored from one final physical state:
the plug resting in the sound socket's well.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=receptacle_plug_move.py -E scene=station -E object_jitter=0 -E control_hz=20 \
  --instruction "the left socket is scorched - move the lamp's plug to the sound socket beside it" \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/plugmove-demo
```

Score a trial by reading the scorer's `task_score` line: 1.0 only when the plug rests in the
right (sound) socket's well at seat height.

## Timing and seeds

- The scripted policy completes the extraction, carry and reseating in ~21 s (420 control
  steps in the verified run), well within the 2,400-step run limit.
- `object_jitter=0` for the canonical layout shown in the setup screenshot; other seeds replay
  the same scene with small layout perturbations.
