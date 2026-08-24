# setup.md — Full Environment Setup From Zero

Reproduce the complete WorldXD system on a fresh machine: ROS 2 + PyTorch/MPS
world model + RViz2 simulation + physics + manual markers + web dashboard +
terminal TUI + test batteries.

**Last updated:** 2026-08-23 · **Primary platform:** macOS Apple Silicon (M1–M4)
**Linux path:** supported with one config edit (§3)

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Install Pixi & Clone](#2-install-pixi--clone)
3. [Platform Configuration (macOS vs Linux)](#3-platform-configuration)
4. [Environment Installation (the `.pixi` folder)](#4-environment-installation)
5. [Model Checkpoints & Caches](#5-model-checkpoints--caches)
6. [Verify the Installation](#6-verify-the-installation)
7. [Run the Full Simulation Stack](#7-run-the-full-simulation-stack)
8. [Web Dashboard Setup](#8-web-dashboard-setup)
9. [Terminal TUI (`wxd`)](#9-terminal-tui)
10. [Manual Control in RViz2](#10-manual-control-in-rviz2)
11. [Prompt Grammar Quick Reference](#11-prompt-grammar)
12. [Run the Test Batteries](#12-run-the-test-batteries)
13. [Development Workflow](#13-development-workflow)
14. [Troubleshooting Encyclopedia](#14-troubleshooting)
15. [Hardware Roadmap (NEMA17 + SG90)](#15-hardware-roadmap)

---

## 1. Prerequisites

| Requirement | Version | Why | Install |
|---|---|---|---|
| **Pixi** | latest | Everything else comes through it | see §2 |
| **Node.js** | ≥ 18 | Web dashboard backend (Express/Socket.IO) | `brew install node` (macOS) / `apt install nodejs npm` (Linux) |
| **Git** | any | clone + workflow | `brew install git` / preinstalled |
| **RAM** | 16 GB min | JEPA fp16 (~3.5 GB) + ROS 2 + RViz2; 8 GB will swap-thrash | — |
| **Disk** | ~10 GB free | `.pixi` env ≈ 6 GB + checkpoint ≈ 1 GB + swap headroom | — |
| **GPU** | Apple M-series (MPS) or CUDA GPU or CPU-only* | model inference | — |

\* CPU-only works but CEM inference is slower; set `device='cpu'` in
`stacking_controller.py` and `test_jepa.py`.

You do **NOT** need: system ROS 2, system Python packages, conda, CUDA toolkit.
Everything lives inside the project-local `.pixi` environment.

---

## 2. Install Pixi & Clone

```bash
# Pixi (user-local, no sudo)
curl -fsSL https://pixi.sh/install.sh | bash
# restart your shell, then verify:
pixi --version

# Clone
git clone https://github.com/Ojas-sta/WorldXD.git
cd WorldXD
```

> After install, `pixi` must be on PATH (`~/.pixi/bin`). If `pixi: command not found`
> in scripts, `export PATH="$HOME/.pixi/bin:$PATH"`.

---

## 3. Platform Configuration

The repo ships pinned to macOS ARM. For other platforms, edit `pixi.toml` line 7:

```toml
# macOS Apple Silicon (as shipped):
platforms = ["osx-arm64"]

# Linux x86_64 (Ubuntu 22.04+):
platforms = ["linux-64"]

# Linux ARM64 (e.g. Jetson, Asahi):
platforms = ["linux-aarch64"]

# Multi-platform:
platforms = ["osx-arm64", "linux-64"]
```

Channels are already correct for all three: `conda-forge` + `robostack-staging`
(RoboStack distributes ROS 2 Humble as conda packages per-platform).

**Apple Silicon note:** MPS acceleration is automatic. On Linux/NVIDIA you may
switch `jepa_model.py` to `device='cuda'`; on CPU-only use `device='cpu'`.
fp16 is used everywhere; both CUDA and MPS support it.

---

## 4. Environment Installation

This creates the project-local `.pixi/` environment folder (~6 GB) containing
Python 3.12, PyTorch, full ROS 2 Humble desktop, RViz2, OpenCV, and every Python
dependency (textual, rich, transformers stack, etc.):

```bash
pixi install          # first run takes 5–20 min depending on bandwidth
```

What gets installed (from `pixi.toml`):

- **ROS layer:** `ros-humble-desktop`, `robot_state_publisher`, `joint_state_publisher`,
  `interactive_markers`, `rviz2`, `xacro`, `cv_bridge`
- **ML layer:** `pytorch` (MPS/CUDA), `torchvision`, `tensordict`, `torchrl`
- **Python deps (pypi):** `huggingface_hub`, `timm`, `einops`, `omegaconf`, `hydra-core`,
  `textual`, `rich`, plus sim/research extras already pinned in the lockfile

> **Never** run these scripts with system Python. Always `pixi run python3 ...`.
> The env folder `.pixi/` is gitignored and fully reproducible from `pixi.toml` +
> `pixi.lock` (the lockfile pins exact builds — do not delete it).

Activate a persistent shell instead of prefixing every command:

```bash
pixi shell            # equivalent to sourcing the env
python3 --version     # now the pixi Python
ros2 topic list       # ros2 CLI available
```

---

## 5. Model Checkpoints & Caches

First model load auto-downloads (~1 GB total):

| Artifact | Source | Lands in |
|---|---|---|
| JEPA-WMS metaworld checkpoint | HuggingFace `facebook/jepa-wms` | `~/.cache/huggingface/hub/models--facebook--jepa-wms/` |
| DINOv2 ViT-S/14 decoder weights | fbaipublicfiles.com direct URL | `~/.cache/torch/hub/checkpoints/` |
| DINOv2 vision transformer code | github facebookresearch/dinov2 | `~/.cache/torch/hub/facebookresearch_dinov2_main/` |

No tokens or accounts required. To pre-warm without launching anything:

```bash
pixi run python3 test_jepa.py    # downloads on first run, then verifies the planner
```

---

## 6. Verify the Installation

Run in order; stop at the first failure and consult §14:

```bash
# 1. ROS 2 alive inside pixi?
pixi run ros2 --help

# 2. Model loads + CEM plans (no ROS needed, ~60s incl. download):
pixi run python3 test_jepa.py
#    expect: "PASS: model loaded and CEM planner produced an output"

# 3. Prompt parser battery:
pixi run python3 test_prompt_edge_cases.py        # expect: TOTAL=96 PASS=96

# 4. Goal renderer:
pixi run python3 test_goal_renderer_property.py   # expect: TOTAL=332 FAIL=0
```

---

## 7. Run the Full Simulation Stack

One command spawns everything:

```bash
pixi run python3 launch_robot.py
```

Processes spawned:

| Process | Role |
|---|---|
| `robot_state_publisher` | URDF → TF chain for RViz2 robot model |
| `workspace_env.py` | 4 colored blocks, physics engine, synthetic 30 Hz camera |
| `stacking_controller.py` | FSM + IK + async JEPA worker + prompt parsing |
| `manual_marker.py` | RViz interactive markers (EE jog + draggable blocks) |
| `rviz2 -d robot_description/rviz/config.rviz` | 3D visualization |
| `terminal_prompt.py` | Tkinter prompt window |

Closing the RViz2 window shuts down the whole stack.

**What you should see:** EEZYbotARM MK2 in RViz2, four ArUco-tagged cubes on the
table, and a live top-down camera view. Verify topics in another terminal:

```bash
pixi run ros2 topic hz /camera/image_raw --window 30   # expect ~30 Hz
pixi run ros2 topic echo /fsm_state --once             # expect "data: DONE"
```

Then give it a task (any interface — Tkinter window, dashboard, TUI, or CLI):

```bash
pixi run ros2 topic pub --once /user_prompt std_msgs/msg/String \
  "{data: 'pick up the yellow block and place it on top of the green block'}"
```

Watch the arm execute approach → grasp → lift → transport → stack (~9 s), then check:

```bash
strings /var/folders/rw/*/T/opencode/*.log 2>/dev/null | grep -E "Grabbed|Released"
# or simply watch RViz2: yellow ends up exactly one block-height above green
```

---

## 8. Web Dashboard

Two processes in addition to the sim (Node.js ≥ 18 required):

```bash
# Terminal A — telemetry relay (ROS -> HTTP):
pixi run python3 dashboard/backend/ros_bridge.py

# Terminal B — API + UI server (port 4002):
cd dashboard/backend && node server.js
```

Open **http://localhost:4002** — the backend serves the built React frontend itself.

Features:
- **Robot Dispatcher**: quick buttons (Arrange All, Reset), color-pair builder,
  free-text prompts — all publish to `/user_prompt` via pixi-routed ros2
- **Onboard Camera**: live base64-JPEG feed of the synthetic camera (~8 fps)
- **State Machine**: live chip pipeline lighting up through
  Approach → Descend → Grip → Lift → Transport → Place → Release → Retreat
- **Model card / System stats / Log stream**: real inference latency, joint angles,
  block positions, process list (no fabricated metrics)

Frontend rebuild (only if you modify `dashboard/frontend/src`):

```bash
cd dashboard/frontend && npm install && npx vite build
```

> Backend `node_modules` are committed; if missing: `cd dashboard/backend && npm install`.

**Port hygiene:** nothing else may hold :4002. Check with `lsof -ti :4002`;
a stale server silently serves old code/fake data.

---

## 9. Terminal TUI

```bash
pixi run wxd
```

Full-screen control center: ASCII banner, live FSM pipeline with step highlighting,
joint angles, gripper state, CEM latency, blocks table, scrolling event log, and a
prompt input box that publishes directly to `/user_prompt`.

| Key | Action |
|-----|--------|
| `a` | arrange all blocks |
| `r` | reset scene |
| `g` | pick green → place on yellow |
| `s` | save camera frame to `captures/` |
| `p` | focus the prompt box |
| `q` | quit |

Requires a real terminal (not piped output). Tested with Terminal.app/iTerm2 on macOS
and standard Linux terminals.

---

## 10. Manual Control in RViz2

Spawned automatically by `launch_robot.py` (`manual_marker.py`):

- **Jog the arm:** drag the cyan sphere (or its axis arrows). The controller enters
  `MANUAL` state and tracks your hand at a safe speed.
- **Gripper:** right-click the sphere → *Toggle gripper* (works while jogging).
- **Move blocks:** drag any colored cube; the world updates and gravity re-settles it.
- **Guardrails:** while an autonomous task runs, all markers become non-draggable and
  manual commands are refused at three layers (marker, relay, controller).
- **Auto-return:** MANUAL expires 3 s after your last drag; the arm retreats home.

Physics is live: drop a block mid-air and it falls; lower it onto another and it
stacks; support <50 % of its footprint and it tips off; yank a base block and towers
collapse; arm links shove blocks aside (only the 5.5 cm gripper zone can "touch" for
pickup).

---

## 11. Prompt Grammar

```
precedence: reset > arrange > pick-task > unrecognized

reset intent:    reset | clear | stop | separate | unstack | split
arrange intent:  arrange ... | everything ... | all blocks | each other | one tower
pick task:       (pick [up] | grab | move | take | lift | place) the <color> block
                 [+ destination: (on top of | onto | on | over | above) the <color>]
colors:          red | green | blue | yellow      (self-stack destination ignored)
```

Examples that all work: `PICK UP THE RED BLOCK` · `grab green, place over blue` ·
`the red block should go on top of blue` · `put everything on top of yellow` →
arrange · `unstack them` → reset. Unknown colors/verbs → polite unrecognized reply.
96-case grammar proof: `teste.md` §2.

---

## 12. Run the Test Batteries

489 cases across four suites (details + evidence: `teste.md`):

```bash
pixi run python3 test_prompt_edge_cases.py           # 96 parser cases      (~2s)
pixi run python3 test_goal_renderer_property.py      # 332 renderer checks  (~5s)
pixi run python3 test_jepa_robustness.py             # 19 hostile inputs    (~3min)
pixi run python3 test_physics_edge_sweep.py          # 42 live sim cases    (~12min)
```

All exit `0` on success (CI-ready). The physics sweep requires the sim running (§7).

---

## 13. Development Workflow

Documented authoritatively in `Onboarding.md`. Short form:

```
① PLAN (state milestone P#, files, success criteria; get human approval)
② IMPLEMENT (match conventions; syntax-check before running)
③ TEST & VERIFY (capture verbatim before/after logs; log-proof or it didn't happen)
④ DOCUMENT + COMMIT + PUSH (Milestones_Log.md 4-part entry w/ date+time)
```

Documentation obligations per change:

| Change type | Update |
|---|---|
| Milestone / bug-fix session | `Milestones_Log.md` (4-part entry) |
| New/changed tests | `teste.md` |
| Any UI work | follow `design-scheme.md` |
| Workflow rule changes | `Onboarding.md` |
| Deps / architecture | `README.md` + `pixi.toml` + `pixi.lock` (commit lockfile!) |

Stable files (do not touch without stating why): `launch_robot.py`,
`robot_description/urdf/eezybotarm.urdf`.

Commit style: one commit per logical unit; message states what AND why; never commit
unverified work; push after every documented milestone.

---

## 14. Troubleshooting Encyclopedia

Every item below has actually happened in this project:

| Symptom | Cause | Fix |
|---|---|---|
| `RuntimeError: LayerNorm((400,)) expected … 384` | proprio dropped before AdaLN predictor | always pass dict `{"visual":…, "proprio":…}` to `encode()`/`unroll()` |
| `'NoneType' object is not subscriptable` from unroll | plain-dict input fell through return chain | fixed in vendored `vit_enc_preds.py:373`; keep the fix when updating the clone |
| `Cannot convert a MPS Tensor to float64` | fp64 tensor reached MPS | fixed via fp32 coercion in `get_action`; keep new tensors off fp64 |
| `Expected all tensors … mps:0 and cpu` | mixed-device math (stats vs activations) | keep denorm/stats on the model device; `.cpu()` only at the boundary |
| `ImportError: cannot import name 'Float32' from 'sensor_msgs.msg'` | wrong module | `Float32` lives in `std_msgs.msg` |
| `ros2: command not found` (in Node childprocs/scripts) | ros2 exists only inside pixi | route through `pixi run ros2 …` with correct `cwd` |
| Dashboard serves old/fake data; clicks do nothing | stale server squatting :4002 | `lsof -ti :4002 \| xargs kill`; restart `node server.js` |
| System swaps hard, CEM "hangs" for many minutes | 16 GB thrashing from stale sims | kill all sim processes, relaunch clean; prefer reduced CEM (64×2) live |
| Blocks flicker/vanish from camera feed | per-frame ArUco generation at odd sizes (fixed) | keep the 128px pre-rendered marker cache pattern |
| Feed stuck at 10 Hz | old timer constant | must be `create_timer(1.0/30.0)` |
| Arm barely moves during tasks | synchronous inference starving executor | JEPA must stay on its background worker thread |
| Gripper grabs the wrong block | legacy first-match 6 cm radius | nearest-within-4cm logic is in `workspace_env.gripper_callback` |
| Block floats mid-air with tiny support | pre-P3.6.2 surfacing bug | tip-off branch in `physics_step()`; don't regress it |
| Arm clips through blocks | arm-collision disabled or tower too short for link plane | links collide above single-block height; verify with 2-block towers |
| RViz drops ~1 camera frame per 2.5 s | cross-node clock jitter (cosmetic) | backdated stamps reduce it; harmless otherwise |
| URDF meshes missing on a new machine | absolute `file:///Users/roopalisingh/...` mesh paths | known limitation — replace with `package://` paths or symlink the CAD dir to the same absolute path |
| TF lookups fail right after launch | DDS discovery lag | wait 5–10 s; long-lived nodes self-heal |
| wxd crashes with `BadIdentifier: '#left'` | class-name syntax | classes take bare words; `#` is for ids (already fixed) |

---

## 15. Hardware Roadmap

Target rig (milestone **P6**, not yet implemented): **EEZYbotARM MK2** driven by
**4 × NEMA17 steppers** + **SG90 servo** gripper.

Planned bridge design (so sim and hardware share interfaces):

- One hardware node subscribes to the existing `/joint_states` and `/gripper_closed`
  topics — zero changes to controller/dashboard/TUI
- NEMA17 via A4988/TMC2209 drivers (step/dir), limit switches for homing
- EEZYbotARM joint2/joint3 are linear push-rods → non-linear angle↔travel mapping
  needs calibration tables
- SG90: 50 Hz PWM, 1–2 ms duty; map `gripper_open` bool → ~0°/90°
- Recommended microcontroller bridge: Arduino running a tiny serial protocol, or
  `ros2_control` with a custom GPIO hardware interface

Until P6 lands, everything above runs in simulation only.

---

*After your first successful `launch_robot.py`, type "arrange all blocks" and watch it work.*
