# WorldXD — Milestones Log

**Project:** AI Robotic Arm Simulation (EEZYbotARM MK2 × JEPA-WMS world model)
**Repo:** https://github.com/Ojas-sta/WorldXD
**Machine:** MacBook Air M3, 16GB RAM, macOS · **Runtime:** pixi (Python 3.12, ROS2 Humble, PyTorch/MPS)
**Hardware target (declared):** NEMA17 stepper motors on all axes + SG90 servo for gripper

---

## Milestone Index

| ID | Milestone | Status |
|----|-----------|--------|
| P0 | Verify JEPA-WMS CEM planner fixes end-to-end | ✅ Done — 2026-08-23 |
| P1 | Fix glitchy synthetic camera feed | ✅ Done — 2026-08-23 |
| P2 | Add `/camera/camera_info` publisher | ✅ Pre-existing (doc was stale) |
| P3 | Stack-boxes UI (buttons in prompt GUI) | ⬜ Not started |
| P4 | Close the loop: JEPA drives joints directly | ⬜ Not started |
| P5 | CEM performance optimization (~14s → real-time) | ⬜ Not started |
| P6 | Real hardware bridge (NEMA17 steppers + SG90 servo) | 🆕 Proposed |

---

## Session 2026-08-22/23 — "Verification & Pick-and-Place"

Work performed by Antigravity (AI) with Roopali Singh.

### 0. Repo setup — `1d7390e` @ 2026-08-22 23:48

**What:** Initialized git repo, created public GitHub remote, initial commit of entire
project including the vendored `jepa-wms/` clone (with the local `unroll()` dict fix baked in).

- **Why:** Project was entirely local/unversioned; needed history + backup before risky refactors.
- **Files:** everything (382 files); added `.gitignore` (`.pixi/`, `__pycache__/`, `.DS_Store`, checkpoints).
- **Notes:** `jepa-wms/` was vendored by removing its nested `.git` so our one-line fix is tracked.
- **Commit:** [`1d7390e`](https://github.com/Ojas-sta/WorldXD/commit/1d7390e)

### P0. JEPA CEM planner verified end-to-end — `7c1ce02` @ 2026-08-23 00:30

**What:** Recreated the lost isolated test (`analyze_jepa.py` had been deleted with an old
scratch directory) as `test_jepa.py`. Confirmed both critical fixes work:

- ✅ Bug 3 fix: `unroll()` returns dict when given dict input (`vit_enc_preds.py:373`)
- ✅ Bug 4 fix: proprio passed through AdaLN predictor → no more LayerNorm(400)-vs-384 crash
- Result: `CEM Planner returned action: [0.0872, 0.1, 0.0345, ...]` (20 floats, non-zero), 14.2s at reduced size (64 samples × 2 iters).

**Bugs found while verifying:**

1. **Doc error:** metaworld checkpoint is trained at **224×224** (`img_size: 224` in YAML),
   not 256×256 as the onboarding doc claimed. Controller was already correct.
2. **Action mapping confirmed OK:** `get_action()` returns 20 floats
   (4 raw actions × frameskip=5 flattened); controller reading indices 0–3 correctly gets
   the immediate next raw action.

**Changes:**
- `test_jepa.py` *(new)* — standalone verification script, no ROS2 required; replicates the exact tensor pipeline of the controller.
- `jepa_model.py` — `JEPAWorldModel.__init__` now accepts `num_samples=256, iterations=3`
  kwargs (defaults unchanged) so tests can run light without editing source.

- **Why:** P0 gate — nothing else could be trusted until the model ran clean.
- **Commit:** [`7c1ce02`](https://github.com/Ojas-sta/WorldXD/commit/7c1ce02)

> ⚠️ Ops note: first full-size CEM run appeared hung for ~20 min. Root cause was **memory
> thrashing**: 13.4 GB / 14 GB swap used with a stale full sim still running in background.
> Killed stale processes and reduced CEM size for verification. Full-size perf remains open (P5).

### P1. Glitchy camera feed fixed + live pick-and-place works — `8e76519` @ 2026-08-23 15:29

**What:** Diagnosed why the arm barely moved after prompts ("minute movement" in RViz),
then fixed that plus four related defects. End result: full pick-and-place executes in ~9s.

**Bugs found & fixed:**

| # | Symptom | Root cause | Fix |
|---|---------|-----------|-----|
| 1 | Arm movement tiny/frozen | JEPA inference (~14s) ran synchronously inside `image_callback`, starving the single-threaded executor → 30Hz `control_loop` got one tick per inference | Moved JEPA to a background worker thread; callback only stashes newest frame |
| 2 | Gripper grabbed wrong block (red instead of green) | Grasp check took *first* block within 6cm; adjacent blocks sit 5cm apart | Grab **nearest** block within 4cm (`workspace_env.py`) |
| 3 | Arm stuck mid-air after grasp | LIFT target recomputed *relative* to moving EE every tick → goal receded forever | Capture absolute lift Z once on state entry |
| 4 | Random block flicker in feed | Per-frame `generateImageMarker()` fails at tiny/odd sizes; blanket `except: pass` hid it | Pre-render ArUco markers once at 128px, resize per frame; clamp projection size 4–224px; log exceptions (rate-limited) |
| 5 | Feed actually 10Hz not 30Hz | Timer at `0.1`s | `1.0/30.0`; measured 30.0Hz ±2ms after fix |
| 6 | Model silently demoted to CPU | `.to(self.device)` where device='cpu' moved MPS-loaded model back | Removed `.to()`; controller uses `model.device` |
| 7 | Periodic RViz frame drops (~1 per 2.5s) | Image stamps marginally older than TF cache (cross-node clock jitter) | Backdate image/camera_info stamps 50ms *(partially effective — cosmetic, still occasionally drops)* |

**State machine implemented** (`stacking_controller.py` rewritten):

```
DONE → MOVE_ABOVE_BLOCK → DESCEND → CLOSE_GRIPPER → LIFT
     → MOVE_ABOVE_STACK → PLACE → OPEN_GRIPPER → RETREAT → DONE
```

Driven geometrically via TF lookups; gripper settle pauses (~0.6s); EE speed limit 0.06 m/s;
arrival tolerance 6mm; task queue for "arrange all".

**Prompt parsing extended:**
- `"pick up the green block and place it on top of the yellow block"` ✓ (color→ID map, self-stack rejected)
- `"arrange all"` → sequential stacking queue ✓
- `"reset"` ✓

**Verified live (ROS logs):**
```
Task: pick block 1 at [0.2, 0.1, 0.02] -> block 3
Closing gripper.            → workspace_env: "Grabbed block 1"   ✓ correct block
Opening gripper.            → workspace_env: "Released block 1"  ✓ on yellow
Task complete: placed block 1.                                    (~9s total)
```

**Files changed:** `stacking_controller.py` (rewritten), `workspace_env.py`,
`jepa_model.py`, plus dashboard scaffolding from a parallel session included in this commit.
- **Why:** The core promise of the project — natural language → physical stacking motion — now works in simulation.
- **Commit:** [`8e76519`](https://github.com/Ojas-sta/WorldXD/commit/8e76519)

---

## Open Items

| ID | Item | Notes |
|----|------|-------|
| P3 | Prompt GUI buttons | `terminal_prompt.py` is text-only today |
| P4 | JEPA drives joints directly | Blocked on: real rendered goal images (currently black placeholder), action normalization vs training distribution (outputs saturate ±0.1), feeding real joint angles as proprio instead of zeros |
| P5 | CEM speed | 64×2 ≈ 14s; 256×3 much worse. Options: fewer samples, batched MPS ops, torch.profiler, or CPU fallback comparison |
| P6 | Hardware bridge | NEMA17 (A4988/TMC2209 drivers, homing switches, non-linear joint2/3 push-rod mapping) + SG90 PWM gripper. Node subscribes to existing `/joint_states` + `/gripper_closed` so sim/hardware share interfaces |
| — | RViz frame drops | Cosmetic; revisit TF clock alignment if it matters |
| — | URDF portability | Mesh paths are absolute `file:///Users/roopalisingh/...` — breaks on other machines |
