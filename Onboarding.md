# WorldXD — Onboarding Guide (How We Work)

**Read this first if you are a human contributor or an AI agent picking up this project.**
For *what* was built and *when*, see `Milestones_Log.md`. For architecture deep-dives and
debugging lessons, see the full technical onboarding doc in git history / session notes.

---

## 1. The Project in One Paragraph

A simulated EEZYbotARM MK2 (ROS2 Humble, RViz2) driven by natural-language prompts.
Meta's JEPA-WMS world model watches a synthetic camera feed and plans actions via CEM;
a geometric IK + state-machine controller currently performs pick-and-place of colored
blocks. Everything runs locally via **pixi** on a MacBook Air M3 (MPS backend).
Hardware target: NEMA17 steppers (all axes) + SG90 servo (gripper).

## 2. Golden Rules

1. **Everything runs through pixi.** Never use system Python.
   ```bash
   pixi run python3 <script>.py        # any script
   pixi run python3 launch_robot.py    # full sim (RViz, GUI, controller, world)
   pixi run ros2 topic list            # ROS2 CLI
   ```
2. **Never commit secrets**, never force-push, only commit when work is verified.
3. **Don't touch stable files without stating why:** `launch_robot.py`,
   `robot_description/urdf/eezybotarm.urdf`.
4. **No new milestone work until the current one is tested, documented, committed & pushed.**

## 3. The Workflow (per unit of work)

```
┌─────────┐   ┌───────────────┐   ┌──────────┐   ┌─────────────────────┐
│ 1 PLAN  │ → │ 2 IMPLEMENT   │ → │ 3 TEST & │ → │ 4 DOCUMENT + COMMIT │
│         │   │               │   │ VERIFY   │   │   + PUSH            │
└─────────┘   └───────────────┘   └──────────┘   └─────────────────────┘
```

### Step 1 — Plan
- State the goal, which milestone (P0–P6) it belongs to, files expected to change,
  and how success will be measured.
- Get explicit human approval before writing code.

### Step 2 — Implement
- Mimic existing style; check imports/conventions of neighboring code.
- No speculative features; no comments unless they explain *why*.
- Syntax-check before running: `pixi run python3 -c "import ast; ast.parse(open('<f>').read())"`.

### Step 3 — Test & Verify
- Run the relevant test path (`test_jepa.py` for model-only, full sim for integration).
- **Capture actual logs before/after** — these go into `Milestones_Log.md` (see §4).
- A fix is not done because the code changed; it is done when the log proves it.

### Step 4 — Document + Commit + Push
- Update `Milestones_Log.md` (structure below), commit with a descriptive message,
  push to `origin/main`.

## 4. What to Document, and Where

| Artifact | Lives in | When |
|----------|----------|------|
| Milestone progress: plan, implementation, old-vs-new test logs, overview | `Milestones_Log.md` | Every milestone or bug-fix session |
| How we work, documentation rules, conventions | `Onboarding.md` (this file) | Rarely; only when workflow changes |
| Setup, dependencies, architecture | `README.md` | When deps/architecture change |
| Commit messages | Git history | One commit per logical unit, message states what & why |

### Required structure for every milestone entry in `Milestones_Log.md`

```markdown
# P<n> — <Title>
**Commit:** <hash+link> · <date>

## ① Plan          ← goal, scope, success criteria (written BEFORE implementing)
## ② Implementation← what changed, file by file, and WHY
## ③ Test & Verification
   ### OLD logs    ← captured broken behavior (verbatim)
   ### NEW logs    ← captured working behavior (verbatim)
## ④ Overview      ← 2-4 sentences: what is now true that wasn't before
```

Rules for test logs:
- Always paste **real terminal output**, not paraphrases.
- Always show OLD vs NEW side by side so regressions are obvious later.
- Include ops notes (e.g., "run appeared hung → machine was swapping") — future you
  will hit the same wall.

## 5. Key Technical Facts (memorize these)

| Fact | Value |
|------|-------|
| Model input size | 224×224 (NOT 256) — `img_size: 224` in eval YAML |
| JEPA action output | 20 floats = 4 raw × frameskip=5; indices 0–3 = next raw action |
| Proprio | must be passed to `unroll()` or AdaLN LayerNorm(400) crashes |
| CEM params | default 256 samples/3 iters (~minutes); light mode 64/2 (~14s); kwargs on `JEPAWorldModel` |
| Device | MPS fp16, loaded via local clone `torch.hub.load(..., source='local')`; NEVER call `.to(device)` on the wrapper afterwards |
| Camera topics | `/camera/image_raw` + `/camera/camera_info`, 30Hz synthetic render |
| Control topics | `/joint_states` (4 joints @30Hz), `/gripper_closed` (`Bool`), `user_prompt` (`String`) |
| Grasp rule | nearest block within 4cm of `manipulator_link`, on rising gripper-close edge |
| Known perf trap | 16GB RAM swaps hard if stale sims accumulate — `pkill -f launch_robot.py` between runs |

## 6. Common Commands

```bash
pixi run python3 launch_robot.py                    # full stack (RViz + GUI + nodes)
pkill -f "launch_robot.py"; pkill -f rviz2          # clean shutdown of stale sims
pixi run python3 test_jepa.py                       # isolated JEPA verification (no ROS)
pixi run ros2 topic pub --once /user_prompt std_msgs/String "{data: 'reset'}"
pixi run ros2 topic hz /camera/image_raw --window 30
strings <logfile> | grep -E "Grabbed|Released|Task" # sim logs contain binary noise
```
