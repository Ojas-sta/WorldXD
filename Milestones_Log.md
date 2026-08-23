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
| P0 | Verify JEPA-WMS CEM planner fixes end-to-end | ✅ Done — 2026-08-23 15:29 |
| P1 | Fix glitchy synthetic camera feed + frozen-arm bugs | ✅ Done — 2026-08-23 15:29 |
| P2 | Add `/camera/camera_info` publisher | ✅ Pre-existing (doc was stale) |
| P3 | Stack-boxes UI: dashboard truth-fix, prompt buttons, camera feed, wxd CLI | ✅ Done — 2026-08-23 15:29 |
| P4 | Close the loop: JEPA drives joints directly | ⬜ Not started |
| P5 | CEM performance optimization (~14s → real-time) | ⬜ Not started |
| P6 | Real hardware bridge (NEMA17 steppers + SG90 servo) | 🆕 Proposed |

---

# P0 — Verify JEPA-WMS CEM planner fixes end-to-end

**Commit:** [`7c1ce02`](https://github.com/Ojas-sta/WorldXD/commit/7c1ce02) · 2026-08-23 00:30 00:30

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

**Commit:** [`8e76519`](https://github.com/Ojas-sta/WorldXD/commit/8e76519) · 2026-08-23 15:29 15:29

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

# P3 — Control UI: Dashboard truth-fix, prompt buttons, camera feed, wxd TUI

**Commits:** [`a983621`](https://github.com/Ojas-sta/WorldXD/commit/a983621) 15:55 (design scheme) · [`4e99f06`](https://github.com/Ojas-sta/WorldXD/commit/4e99f06) 16:08 (backend relay) · [`11a0dcf`](https://github.com/Ojas-sta/WorldXD/commit/11a0dcf) 16:12 (frontend) · [`4c9eb74`](https://github.com/Ojas-sta/WorldXD/commit/4c9eb74) 16:17 (CLI)

## ① Plan

Original P3 ("buttons in the prompt GUI") was expanded by user request after a dashboard
audit revealed the existing React+Express dashboard was **mostly fake**: EE position was
random jitter, model stats hardcoded, "GPU TFLOPS" fabricated arithmetic, log stream read
a deleted `.gemini/antigravity` path, FSM names didn't match the real controller, and no
camera feed existed. Expanded scope into four workstreams:

- **WS0** — save the Apple-design skill verbatim as `design-scheme.md`; all UI work obeys it
- **WSA** — real ROS→dashboard telemetry (Node Express kept; new Python rclpy relay)
- **WSB** — frontend: prompt buttons + color-pair builder, live camera feed, FSM pipeline
- **WSC** — `wxd`: full-screen Textual TUI control center (claude-code-style)

## ② Implementation

**`design-scheme.md` (new):** Apple fluid-interface rules. Applied: pointer-down dispatch
on buttons (§1), critically damped springs only (§4), reduced-motion cross-fade fallbacks
(§14), specific labels ("Approach/Descend/Grip..." not generic chips) (§16).

**Controller (`stacking_controller.py`):** publishes `/fsm_state` on transitions and
`/jepa_telemetry` (real CEM inference ms) from the background worker.

**`dashboard/backend/ros_bridge.py` (new):** rclpy node subscribing to `/joint_states`,
`/gripper_closed`, `/fsm_state`, `/jepa_telemetry`, `/workspace_blocks`, `/camera/image_raw`
(8fps JPEG throttle); POSTs JSON to Express. Snapshots carry `fsmStamp` so the server can
reject out-of-order deliveries.

**`dashboard/backend/server.js`:** `/ros/state` + `/ros/camera` endpoints; FSM updates
guarded by monotonic stamp; prompts now published via `pixi run ros2 topic pub ...`
(bare `ros2` doesn't exist outside pixi — was failing silently); deleted all fabricated
metrics and the dead log path; event log now fed by real prompt/FSM events.

**Frontend:** `ControlPanel.tsx` rewritten (Arrange All / Reset / color-pair builder with
grammar matching the controller's regex parser); `CameraFeed.tsx` (new, base64-JPEG +
stale detection); `FsmPipeline.tsx` (new, live state chips); App socket lifted to state;
types updated to real CEM config.

**`wxd.py` (new) + pixi deps:** Textual TUI — ASCII banner, status panel (FSM pipeline,
joints°, gripper, CEM latency), blocks table, scrolling event log, prompt input bound to
an embedded rclpy node, keybindings `a/r/g/s/p/q`. Runs via `pixi run wxd`.

Bugs found during implementation (all fixed):
1. `Float32` imported from `sensor_msgs` — wrong module, controller crashed at startup
2. Stale server from a parallel session held port 4002 → served old fake data
3. Out-of-order HTTP snapshots duplicated FSM transition logs → `fsmStamp` guard
4. Server-side `exec('ros2 ...')` failed silently outside pixi env
5. Textual rejects `classes='#left'` (`#` is id syntax)
6. Review-flagged frontend regressions from parallel session: dropped pulse animation,
   process-list slice mismatch, misleading "60 FPS" badge — fixed in WSB commit

## ③ Test & Verification

**OLD logs (broken/fake dashboard):**

```
# /api/state served fabricated data (stale parallel-session server on :4002):
fsm: DONE
joints: [0, -0.21, 0.45, -0.24]          ← hardcoded constants
blocks: [hardcoded initial positions]     ← never updated
inferenceMs: 31.5                         ← invented number
cemConfig: {numSamples: 256, iterations: 3}   ← didn't match running code (64/2)
# Log stream source path deleted months ago -> permanently empty.
# Server console: EADDRINUSE :::4002 (old instance blocking updated code)
```

```
# Dashboard-dispatched prompt never reached the robot (ros2 not on PATH):
[4:03:19 PM] Prompt: "pick up the green block ..."
(30+ seconds pass; controller receives nothing; fsm stays DONE)
$ which ros2 → not found                  ← bare exec() inside node failed silently
```

**NEW logs (after fixes):**

Dashboard-dispatched pick-and-place, full real FSM stream, zero duplicates:
```
[4:12:12 PM] Prompt: "pick up the blue block and place it on top of the red block"
[4:12:14 PM] FSM -> MOVE_ABOVE_BLOCK
[4:12:16 PM] FSM -> DESCEND
[4:12:17 PM] FSM -> CLOSE_GRIPPER
[4:12:18 PM] FSM -> LIFT
[4:12:19 PM] FSM -> MOVE_ABOVE_STACK
[4:12:22 PM] FSM -> PLACE
[4:12:23 PM] FSM -> OPEN_GRIPPER
[4:12:23 PM] FSM -> RETREAT
[4:12:26 PM] FSM -> DONE
blocks: [('Red', [0.15, 0.1, 0.02]), ('Green', [0.2, -0.099, 0.05]),
         ('Blue', [0.15, 0.099, 0.05]), ('Yellow', [0.2, -0.1, 0.02])]
                                              ↑ Blue now ON Red at z=0.05 — task really executed
inferenceMs: 10800                            ← measured, not invented
cemConfig: {'numSamples': 64, 'iterations': 2} ← matches running code
camera: jpeg OK, 1410 bytes                   ← live synthetic feed flowing
```

wxd TUI headless pilot-harness test against the live sim:
```
1. compose OK
2. ros data: fsm=DONE joints=[0.03, -1.23, 1.73, -0.5] blocks=4
3. keybinding dispatch OK: prompt: "pick up the green block and place it on top of the yellow..."
4. blocks table rows: 4
ALL WXD TESTS PASSED
# Controller confirms receipt:
Task: pick block 1 at [0.2, -0.099, 0.05] -> block 3
Task complete: placed block 1.
```

## ④ Overview

Every surface that shows robot state now shows *real* state: web dashboard, API, and TUI
all read the same ROS topics through one relay. Prompts can be dispatched three ways
(Tkinter GUI, web dashboard buttons/pair-builder, `wxd` keys/input) and land on the same
verified pick-and-place pipeline. Camera feed is visible in-browser; `pixi run wxd` gives
a full terminal control center. Fabricated metrics are gone — unknowns render as "n/a".

---

# P3.1 — Manual control add-ons: EE jog + draggable blocks (interactive markers)

**Commits:** [`c65b7aa`](https://github.com/Ojas-sta/WorldXD/commit/c65b7aa) 16:24 (EE jog) ·
[`c0b7e26`](https://github.com/Ojas-sta/WorldXD/commit/c0b7e26) 16:31 (draggable blocks)

## ① Plan

User requested manual manipulation in RViz: drag a marker to move the arm, and drag the
dummy blocks directly. Constraints: joint_states must stay single-writer; block state is
owned by workspace_env.

## ② Implementation

- **`manual_marker.py`** — cyan sphere marker streams `/ee_target` while dragged
  (controller follows in MANUAL state); one colored cube per block streams `/block_move`
  (PointStamped with `frame_id=block_<id>`). Block markers re-sync to authoritative state
  except the specific marker mid-drag (1.5s grace), so arm-carries and resets stay
  consistent. Context menu toggles gripper during MANUAL.
- **`stacking_controller.py`** — MANUAL state tracks the live drag target at ee_speed;
  any task prompt/reset exits. Gripper obeys `/manual_gripper` only while MANUAL.
- **`workspace_env.py`** — `/block_move` relocates blocks, clamped to table bounds;
  refused while the arm carries that block.
- **UI:** MANUAL appears as a mode chip in dashboard FsmPipeline and wxd TUI.
- **launch_robot.py / rviz config** updated to spawn + display the markers.

## ③ Test & Verification

```
BEFORE: {0: (0.15, 0.1, 0.02), 1: (0.2, 0.1, 0.02), 2: (0.15, -0.1, 0.02), 3: (0.2, -0.1, 0.02)}
AFTER:  {0: (0.15, 0.1, 0.02), 1: (0.2, 0.1, 0.02), 2: (0.28, -0.18, 0.02), 3: (0.2, -0.1, 0.02)}
DRAG TEST: PASS          ← simulated /block_move stream relocated exactly block 2

controller: MANUAL jog engaged (marker drag).      ← EE jog engages on drag
dashboard log: FSM -> MANUAL ... FSM -> DONE       ← exits cleanly on reset prompt
wxd/dashboard show MANUAL chip live
```

## ④ Overview

The arm can now be jogged by hand in RViz (dragging the cyan target), the gripper toggled
from the marker context menu, and any dummy block picked up and moved by dragging its
cube — with the simulation state, camera feed, dashboard and TUI all staying consistent.

---

# P3.3 — Guardrails: manual control locked while JEPA/tasks run

**Commit:** [`72948d3`](https://github.com/Ojas-sta/WorldXD/commit/72948d3) · 2026-08-23 16:39

## ① Plan

User request: while JEPA/state-machine tasks run, (a) EE jog must not steal the arm,
(b) blocks must be fixed (no interactive-marker moves), (c) the jog sphere's axis arrows
must disappear. Guardrail authority = controller's `/fsm_state`.

## ② Implementation

Three independent layers so a stale client can never win:
1. **`manual_marker.py`** — subscribes `/fsm_state`; on task start re-builds all markers:
   EE sphere dimmed + `interaction_mode=NONE` (axes removed), block cubes non-draggable;
   feedback callbacks additionally drop any drags that slip through. On `DONE`/`MANUAL`
   everything is restored.
2. **`workspace_env.py`** — refuses `/block_move` while task busy (logged, rate-limited).
3. **`stacking_controller.py`** — `/ee_target` ignored unless state is DONE/MANUAL.

## ③ Test & Verification

```
[manual_marker] Guardrails ENGAGED (task running) [fsm=MOVE_ABOVE_BLOCK]
(10x /ee_target drags sent during task -> zero "MANUAL jog engaged" in controller log)
DURING task: yellow still home? True -> (0.2, -0.1)      ← /block_move refused
[manual_marker] Guardrails released [fsm=DONE]
AFTER task done: yellow moved to exactly (0.3, 0.22)     ← draggable again
```

(Note: a "GUARDRAIL TEST: FAIL" line in the raw test output was a sign error in the test
script itself (`y + 0.22` checks for -0.22); the block verifiably reached the target.)

## ④ Overview

Manual control and autonomous tasks can no longer fight over the robot: the moment a
task starts, all interactive markers go read-only and every transport layer drops manual
commands; control returns automatically on completion.

---

# P3.5 — Physics for the dummy blocks (gravity + stacking)

**Commit:** [`see git log`](https://github.com/Ojas-sta/WorldXD/commits/main) · 2026-08-23 17:05

## ① Plan

User request: "add physics in rviz". RViz has no physics engine; PyBullet/MuJoCo (already
in pixi deps) are overkill for four cubes. Chosen scope: lightweight rigid-body-ish
behavior inside workspace_env at the existing 30Hz tick:
- free blocks fall under scaled gravity
- rest on the highest supporting surface (table surface or another block's top)
- hand-dragged blocks stay kinematic while dragged, then physics resumes
- dragging a block into a support column pushes it up (no interpenetration)
- removing a base block collapses anything stacked on it

## ② Implementation

`workspace_env.py`:
- `physics_step()` — per-block velocity integration (`GRAVITY=2.5 m/s²`, readable at 30Hz)
- `_support_height()` — highest surface with XY overlap at/below block center;
  `rest_z = support + BLOCK_HALF`
- drag grace: `/block_move` messages stamp `_last_move[bid]`; blocks are kinematic for
  0.35s after the last message, so RViz drags feel 1:1, then gravity takes over
- grabbed blocks exempt (position owned by gripper TF); reset also zeroes velocities

Bugs hit during implementation (both fixed same commit):
1. Init-order crash: physics state referenced `self.blocks` before creation
   → `AttributeError` on startup.
2. Frame-offset bug: `TABLE_Z` set to 0.02 (the resting *center*) instead of 0.0 (the
   table *surface*) → every block rested exactly 2cm high. Caught by tests T1–T3 all
   failing with a uniform +0.02 offset.

## ③ Test & Verification

First run (offset bug present):
```
T1 blue after release: (0.15, -0.1, 0.04) -> FAIL     ← expected 0.02
T2 red on yellow:      (0.2, -0.1, 0.08)  -> FAIL     ← expected 0.06
T3 red fell to table:  (0.2, -0.1, 0.04)  -> FAIL
```
After TABLE_Z fix:
```
--- T1: FALL ---
blue after release: (0.15, -0.1, 0.02) -> PASS        ← released mid-air, fell to table
--- T2: STACK ON BLOCK ---
red on yellow: (0.2, -0.1, 0.06) -> PASS              ← hovered above yellow, settled on its top
--- T3: TOWER COLLAPSE ---
red fell to table: (0.2, -0.1, 0.02) -> PASS          ← yanked yellow out; red collapsed
yellow moved aside: (0.28, 0.18, 0.02)
```

## ④ Overview

The scene now behaves physically: drop a block and it falls, lower it onto another and it
stacks, pull a base block out and the tower collapses — visible in RViz, in the synthetic
camera feed, and therefore to JEPA. Robot-stacked towers are real stacks now: if the arm
places a block past the edge of the one below, gravity decides.

---

# Open Items

| ID | Item | Notes |
|----|------|-------|
| P4 | JEPA drives joints directly | Blocked on: real rendered goal images (currently black placeholder), action normalization vs training distribution, feeding real joint angles as proprio instead of zeros |
| P5 | CEM speed | 64×2 ≈ 10-14s live; 256×3 much worse. Options: fewer samples, batched MPS ops, torch.profiler, CPU comparison |
| P6 | Hardware bridge | NEMA17 (A4988/TMC2209 drivers, homing switches, non-linear joint2/3 push-rod mapping) + SG90 PWM gripper. Node subscribes to existing `/joint_states` + `/gripper_closed` so sim/hardware share interfaces |
| — | RViz frame drops | Cosmetic (~1 per 2.5s); revisit TF clock alignment if it matters |
| — | URDF portability | Mesh paths are absolute `file:///Users/roopalisingh/...` — breaks on other machines |
| — | Dashboard: real camera source | `CameraFeed` toggle stub exists; wire iPhone ARKit bridge for real feed |
| — | wxd: camera snapshot preview in-TUI | Currently saves to `captures/`; could render half-block preview via textual-image plugin |
| — | Duplicate prompts restart tasks | Each received prompt calls `_start_task` even mid-task; consider ignoring prompts while a task is active or queueing them |
