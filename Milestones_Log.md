# WorldXD — Milestones Log

**Project:** AI Robotic Arm Simulation (EEZYbotARM MK2 × JEPA-WMS world model)
**Repo:** https://github.com/Ojas-sta/WorldXD
**Machine:** MacBook Air M3, 16GB RAM, macOS · **Runtime:** pixi (Python 3.12, ROS2 Humble, PyTorch/MPS)
**Hardware target (declared):** NEMA17 stepper motors on all axes + SG90 servo for gripper

Each milestone below is documented in 4 parts: **① Plan → ② Implementation →
③ Test & Verification (old vs new logs) → ④ Overview**.

---

## Milestone Index

| ID | Milestone | Status |
|----|-----------|--------|
| P0 | Verify JEPA-WMS CEM planner fixes end-to-end | ✅ Done — 2026-08-23 |
| P1 | Fix glitchy synthetic camera feed + frozen-arm bugs | ✅ Done — 2026-08-23 |
| P2 | Add `/camera/camera_info` publisher | ✅ Pre-existing (doc was stale) |
| P3 | Stack-boxes UI (buttons in prompt GUI) | ⬜ Not started |
| P4 | Close the loop: JEPA drives joints directly | ⬜ Not started |
| P5 | CEM performance optimization (~14s → real-time) | ⬜ Not started |
| P6 | Real hardware bridge (NEMA17 steppers + SG90 servo) | 🆕 Proposed |

---

# P0 — Verify JEPA-WMS CEM planner fixes end-to-end

**Commit:** [`7c1ce02`](https://github.com/Ojas-sta/WorldXD/commit/7c1ce02) · 2026-08-23 00:30

## ① Plan

The previous session had fixed two critical crashes (Bug 3: `unroll()` returning `None`
for dict input; Bug 4: LayerNorm 400-vs-384 proprio mismatch), but died before verifying.
The original test script (`~/.gemini/.../scratch/analyze_jepa.py`) no longer existed.

Plan:
1. Recreate an isolated verification script (`test_jepa.py`) that runs the exact tensor
   pipeline used by the controller — no ROS2 needed.
2. Confirm: model loads, CEM planner returns a non-zero action, no exceptions.
3. While at it, resolve open questions from the doc: correct input resolution and how the
   20-dim action output maps to the controller's `[dx,dy,dz,gripper]`.

## ② Implementation

- **`test_jepa.py` (new):** loads `JEPAWorldModel`, builds a synthetic camera frame +
  black goal, runs one full CEM pass, prints action + timing. Mirrors the controller's
  transform pipeline exactly (BGR uint8 → PIL → resize 224 → ImageNet normalize).
- **`jepa_model.py`:** `__init__` now accepts `num_samples=256, iterations=3` kwargs so
  tests can run a light CEM without editing source. Defaults unchanged.

Findings folded into implementation:
- Checkpoint is trained at **224×224** (`img_size: 224` in the eval YAML) — the onboarding
  doc's "256×256" claim was wrong; the controller was already correct.
- Action mapping confirmed: output is 20 floats (4 raw × frameskip=5 flattened);
  indices 0–3 = immediate next raw action, which is what the controller consumes.

## ③ Test & Verification

**OLD logs (before this session's fixes)** — from the prior session:

```
Error in get_action: 'NoneType' object is not subscriptable   ← unroll() returned None (dict case missing)
RuntimeError: Given normalized_shape=[400], expected input with shape [*, 400],
              but got input of size [256, 256, 384]           ← proprio dropped before AdaLN predictor
CEM Planner returned action: [0.0, 0.0, 0.0, 0.0]             ← fallback zero-action on every error
```

**NEW log** — `pixi run python3 test_jepa.py` after fixes (64 samples × 2 iters):

```
Loading official facebook/jepa-wms from local clone...
[INFO] Using hardcoded dimensions for metaworld: action_dim=4, proprio_dim=4
[INFO] loaded pretrained predictor from epoch 50 with msg: <All keys matched successfully>
[INFO] loaded pretrained proprio encoder from epoch 50 with msg: <All keys matched successfully>
[INFO] Loaded encoder and predictor
============================================================
Inference time: 14246 ms
Action length: 20
CEM Planner returned action: [0.0872, 0.1, 0.0345, 0.1, -0.1, -0.1, 0.1, 0.1,
                              -0.1, -0.1, 0.1, -0.1, -0.1, -0.1, 0.0614, ...]
PASS: model loaded and CEM planner produced an output
```

No exceptions; non-zero action. Both old errors gone.

> ⚠️ **Ops note:** the first full-size run (256×3) appeared hung ~20 min. Not a deadlock:
> the machine was swapping hard (**13.4 GB / 14 GB swap**) because a stale full sim was
> still running. Killed stale processes; reduced CEM size for verification. Perf work is P5.

## ④ Overview

P0 gate passed — the JEPA world model loads cleanly on MPS/fp16 and plans valid actions.
Nothing downstream could be trusted until this held. Remaining known gap: outputs saturate
the ±0.1 clamp (normalization vs training distribution — deferred to P4/P5).

---

# P1 — Fix glitchy feed + frozen arm + working pick-and-place

**Commit:** [`8e76519`](https://github.com/Ojas-sta/WorldXD/commit/8e76519) · 2026-08-23 15:29

## ① Plan

Live testing showed: RViz feed glitchy, camera actually at 10 Hz, arm barely moved after
prompts ("minute movement"), gripper grabbed the wrong block, and the arm froze mid-air
after grasping. The state machine states (`IDENTIFY` etc.) were dead code with no
transitions, and prompts like "place on top of yellow" weren't parsed.

Plan:
1. Root-cause the minute movement; make `control_loop` responsive.
2. Implement a real pick-and-place state machine driven by TF lookups.
3. Extend prompt parsing for "pick X / place on top of Y".
4. Fix the grasp logic, render glitches, timer rate, and silent exception swallowing.

## ② Implementation

**`stacking_controller.py` (rewritten):**
- JEPA inference moved to a background worker thread; `image_callback` only stashes the
  newest frame (fixes executor starvation).
- New state machine: `DONE → MOVE_ABOVE_BLOCK → DESCEND → CLOSE_GRIPPER → LIFT →
  MOVE_ABOVE_STACK → PLACE → OPEN_GRIPPER → RETREAT → DONE`; geometric targets via TF,
  EE speed 0.06 m/s, arrival tolerance 6mm, gripper settle pauses ~0.6s, task queue for
  "arrange all".
- Prompt parsing via regex: `pick up <color> … on top of <color>` (self-stack rejected),
  plain color moves, arrange-all, reset.

**`workspace_env.py`:**
- Grasp logic: nearest block within 4cm (was first-match within 6cm — adjacent blocks sit
  5cm apart, so neighbors overlapped).
- ArUco markers pre-rendered once at 128px and resized per frame (per-frame generation
  failed at tiny/odd sizes → flicker); projection size clamped to 4–224px.
- Timer 0.1s → 1/30s (true 30Hz). Image/camera_info stamps backdated 50ms to reduce RViz
  message-filter drops. Silent `except: pass` blocks replaced with rate-limited warnings.

**`jepa_model.py` / `stacking_controller.py`:** removed `.to(self.device)` call that was
silently moving the MPS-loaded model onto CPU; controller now uses `model.device`.

## ③ Test & Verification

**OLD logs (broken behavior):**

Prompt test #1 — arm effectively frozen (JEPA blocking the executor):
```
Received prompt: pick up the green block and place it on top of the yellow block
Executing prompt: Moving block 1
CEM Planner chose action: [-0.1, -0.07489013671875, 0.1, 0.1, ...]   ← saturated ±0.1 clamps
(then nothing — control_loop starved for the duration of each ~14s inference)
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called...
```

Wrong-block grasp (old first-match logic):
```
Closing gripper.
workspace_env: Grabbed block 0        ← picked GREEN, grabbed RED (5cm away, first match)
```

Frozen mid-air (LIFT chase bug):
```
Task: pick block 1 at [0.2, 0.1, 0.02] -> block 3
Closing gripper.
Grabbed block 1
(nothing further for 60s+ — LIFT target receded forever)
```

Feed rate before fix:
```
(no measurement possible at first; code inspection showed create_timer(0.1) = 10Hz)
```

**NEW logs (after all fixes):**

Feed rate — stable 30Hz:
```
$ ros2 topic hz /camera/image_raw --window 30
average rate: 30.001
    min: 0.029s max: 0.038s std dev: 0.00179s window: 30
```

Full pick-and-place (~9 seconds):
```
stacking_controller: Received prompt: pick up the green block and place it on top of the yellow block
stacking_controller: Task: pick block 1 at [0.2, 0.1, 0.02] -> block 3
stacking_controller: Closing gripper.
workspace_env:       Grabbed block 1        ← CORRECT block this time
stacking_controller: Opening gripper.
workspace_env:       Released block 1       ← released on top of yellow (PLACE target)
stacking_controller: Task complete: placed block 1.
```

Render-error logging (previously invisible, now surfaced & rate-limited):
```
workspace_env: Render error for block 0: Lookup would require extrapolation into the past ...
```

## ④ Overview

The core promise of the project works end-to-end: natural language → physical stacking
motion in simulation, with the JEPA world model running as a live parallel observer.
Seven defects fixed across three files. Known leftovers: occasional cosmetic RViz frame
drops, saturated CEM outputs, JEPA not yet driving motion (P4).

---

# Repo setup (pre-milestone)

**Commit:** [`1d7390e`](https://github.com/Ojas-sta/WorldXD/commit/1d7390e) · 2026-08-22 23:48

Initialized git, added `.gitignore` (`.pixi/`, `__pycache__/`, `.DS_Store`, checkpoints),
vendored `jepa-wms/` by removing its nested `.git` so our local `unroll()` fix is tracked,
and pushed everything to the new public repo. **Why:** project was entirely unversioned;
history needed before risky refactors.

---

# Open Items

| ID | Item | Notes |
|----|------|-------|
| P3 | Prompt GUI buttons | `terminal_prompt.py` is text-only today |
| P4 | JEPA drives joints directly | Blocked on: real rendered goal images (currently black placeholder), action normalization vs training distribution, feeding real joint angles as proprio instead of zeros |
| P5 | CEM speed | 64×2 ≈ 14s; 256×3 much worse. Options: fewer samples, batched MPS ops, torch.profiler, CPU comparison |
| P6 | Hardware bridge | NEMA17 (A4988/TMC2209 drivers, homing switches, non-linear joint2/3 push-rod mapping) + SG90 PWM gripper. Node subscribes to existing `/joint_states` + `/gripper_closed` so sim/hardware share interfaces |
| — | RViz frame drops | Cosmetic (~1 per 2.5s); revisit TF clock alignment if it matters |
| — | URDF portability | Mesh paths are absolute `file:///Users/roopalisingh/...` — breaks on other machines |
