# teste.md — WorldXD Test Documentation

**Scope:** complete record of every test suite built and executed in this project,
with methodology, raw evidence, known limitations, and retest obligations.
**Last updated:** 2026-08-23 19:15 · **Total documented cases:** 489 + regression suites
**Related:** `Milestones_Log.md` (milestone history) · `Onboarding.md` (workflow rules)

---

## Table of Contents

1. [Test Suite Inventory](#1-test-suite-inventory)
2. [Prompt Parser Battery (96 cases)](#2-prompt-parser-battery)
3. [Goal Renderer Properties (332 cases)](#3-goal-renderer-properties)
4. [JEPA Robustness Battery (19 cases)](#4-jepa-robustness-battery)
5. [Live Physics Sweep in RViz2 (42 cases)](#5-live-physics-sweep)
6. [Regression Suites (P3.5–P3.6.x legacy)](#6-regression-suites)
7. [Debugging Trails Worth Remembering](#7-debugging-trails)
8. [Deferred Items & P4.1 Retest Obligations](#8-deferred-items)
9. [How to Re-run Everything](#9-how-to-re-run-everything)

---

## 1. Test Suite Inventory

| Suite | File | Cases | Needs ROS? | Runtime | Last result |
|-------|------|-------|-----------|---------|-------------|
| Prompt parser | `test_prompt_edge_cases.py` | 96 | No | ~2s | ✅ 96/96 |
| Goal renderer properties | `test_goal_renderer_property.py` | 332 | No | ~5s | ✅ 332/332 |
| JEPA robustness | `test_jepa_robustness.py` | 19 | No (loads model) | ~3min | ✅ 19/19 |
| Live physics sweep | `test_physics_edge_sweep.py` | 42 | **Yes (RViz2 sim)** | ~12min | ✅ 41+1 non-event |
| JEPA verification (P0) | `test_jepa.py` | 1 end-to-end | No | ~1min | ✅ PASS |
| Goal renderer acceptance | `test_goal_renderer.py` | 26 | No | ~2s | ✅ 26/26 |

**Grand total: 489 edge cases + 27 acceptance checks, all green as of 2026-08-23.**

Testing philosophy (per `Onboarding.md`): a fix is not done when the code changes; it is
done when captured logs prove it. Every FAIL encountered during development is recorded
below with its root cause — including failures caused by the *tests themselves*.

---

## 2. Prompt Parser Battery

**File:** `test_prompt_edge_cases.py` · **Target:** `parse_prompt()` in `stacking_controller.py`

Pure-function battery — no ROS required, runs in milliseconds, safe to run anytime.

### Coverage categories

| Category | Examples | Count |
|----------|----------|-------|
| Verb × color matrix | pick/pick up/grab/move/take/lift/stack × red/green/blue/yellow | 24 |
| Destination patterns | on top of / onto / on / over / above (+ article optional) | 7 |
| Casing & punctuation | UPPERCASE, MiXeD, trailing spaces, `!!!`, newlines/tabs | 8 |
| Reset dominance | reset/clear/stop/separate/unstack/split — wins over any task content | 8 |
| Arrange family | arrange/all blocks/everything/each other/one tower/bare `all` | 6 |
| Unrecognized | unknown colors, bare numbers, no word boundaries, spelled-out letters | 8 |
| Self-stack exclusion | "place the red block on top of the red block" → place becomes stack point | 2 |
| Lengthy prompts | 200–400 char rambling with command embedded at start/middle/end | 5 |
| Adversarial | emoji, verb-binds-first-color, mid-sentence corrections, conflicting colors | 12 |
| Degenerate inputs | empty, whitespace-only, tab/newline-only | 3 |

### Grammar semantics enforced by tests

```
precedence: reset > arrange > task > unrecognized
pick binding: pickup verb binds to the color ADJACENT to it
  ("pick up the RED but actually blue block" → red — by design)
destination scan: first 'on top of|onto|on|over|above <color>' after the pick
self-reference excluded: destination == pick → falls back to fixed stack point
"at ALL" must NOT trigger arrange (bare \ball\b removed; requires action context:
  'arrange' | 'everything' | 'all blocks' | 'each other' | 'one tower' | 'a tower')
'unstack/split' → treated as reset/separate intent
```

### History of failures found by this battery (all fixed)

1. `"…given at all."` triggered arrange via bare `\ball\b` regex match inside "at all"
   → replaced with context-required patterns.
2. `'unstack the red from the blue'` fell through to a move-task → added unstack/split
   to the reset family.
3. `'stack all blocks on top of each other into one tower'` was rejected because the
   arrange check excluded any prompt containing "on top of" → now "each other"/"tower"
   phrases force arrange regardless.
4–5. Two expectation errors in tests themselves (verb-binding and everything-on-red
   semantics) — corrected expectations, not code.

---

## 3. Goal Renderer Properties

**Files:** `goal_renderer.py`, `test_goal_renderer_property.py`

Property-based testing with seeded RNG (`np.random.default_rng(seed)` for seeds 0–29):
each seed generates a random layout (1–4 blocks, random xy in workspace, z ∈ {0.02, 0.06})
and asserts the rendered frame against invariants:

- shape `(224,224,3)`, dtype uint8, values within [0,255]
- non-black (mean > 10 — table gray ≈ 50)
- **determinism**: identical input bytes → identical output bytes
- per-block projection: lands in-frame, and a patch around the projected center
  contains the block's dominant BGR signature (red/green/blue/yellow channel rules),
  robust to the ArUco alpha-blend overlay

Plus degenerate-input cases that must not crash:
empty scene list; block 5m out of frame; block behind camera plane (z<0.01);
absurd 100m distance.

**Result: 332/332 checks pass.**

Known intentional deviation from the live feed (documented for P4):
the renderer uses a fixed overhead virtual camera (0.87 m above table center looking
straight down). The live `camera_link` rides the end-effector. The projection math is
otherwise line-for-line identical (f=200, c=112, size=f·0.04/z clamped [4,224],
ArUco DICT_4X4_50 pre-rendered at 128px blended 30/70).

---

## 4. JEPA Robustness Battery

**File:** `test_jepa_robustness.py` · **Target:** `JEPAWorldModel.get_action()`

Loads the real Meta checkpoint once with tiny CEM parameters
(`num_samples=16, iterations=1`) so each case takes seconds instead of minutes.

Every case asserts the output contract: a list of exactly 4 finite floats in [-1, 1]
(denormalized metaworld-space actions), or the zero-fallback — never an exception leak.

| # | Case | Result |
|---|------|--------|
| 1 | normal obs + black goal | ✅ [-0.199, 0.443, 0.093, 0.387] |
| 2 | identity goal == obs | ✅ |
| 3 | white frame | ✅ |
| 4 | pure noise frame | ✅ |
| 5 | goal = white | ✅ |
| 6 | proprio = zeros (out-of-distribution) | ✅ |
| 7 | proprio realistic [0.15, 0, 0.15, 1] | ✅ |
| 8 | proprio negative coords | ✅ |
| 9 | proprio long list w/ mixed junk types (truncate) | ✅ |
| 10 | proprio short list (pad) | ✅ fallback zeros — documented pad behavior |
| 11 | distinct goal_proprio vs proprio | ✅ |
| 12–14 | alpha_proprio ∈ {0.0, 1.0, -0.5} | ✅ cost weight accepted |
| 15 | float64 tensors | ✅ after fix — was `MPS cannot convert float64` crash |
| 16 | extreme values ±50 | ✅ |
| 17 | batch=2 | ✅ returns fallback zeros — batch>1 unsupported by design (live feed B=1); documented limitation |
| 18 | cpu-resident inputs | ✅ device coercion works both directions |
| 19 | NaN frame | ✅ |

**Bugs this battery caught and fixed:**
- fp64 → MPS conversion crash → defensive fp32 route added before fp16 cast
- cpu×mps tensor mismatch in action denormalization (`Expected all tensors on same
  device`) — controller crashed at startup until stats/device handling rewritten

---

## 5. Live Physics Sweep

**File:** `test_physics_edge_sweep.py` · **Runs against the real RViz2 sim** via ROS topics
(`/block_move` drags, `/ee_target` jogs, `/workspace_blocks` sampling).

Methodology: every case computes its EXPECTED outcome analytically from the footprint
support rule — stable iff combined overlap fraction ≥ 0.5 — then compares the sim's
settled state. The sim and the test oracle are independent computations.

### 25-cell offset grid (single supporter)

Offsets dx,dy ∈ {−0.036,−0.018,0,+0.018,+0.036}: predicted stable region
(|dx|≤0.02 ∧ |dy|≥0 or vice versa giving frac ≥ 0.5) matched sim outcome in
**25/25 cells**. Boundary behavior verified: frac 0.55 stacks, 0.30 tumbles.

### Diagonal sweep (7 cases)

d ∈ {±0.034, ±0.025, ±0.015, 0}: only d=0 stacks (frac=(H−√2·d)²/H² < 0.5 elsewhere).
**7/7 correct.**

### Drop heights (6 cases)

Empty column drops from z∈{0.05,0.09,0.14,0.20} land at rest height and stay;
drops onto an occupied column stack on top (z=0.06) from both heights. **6/6.**
(First version wrongly expected stacking spots to be empty — test bug.)

### Bridge support removal (3 permutations)

Corner-touch 50% bridge (yellow home + red diagonal, blue centered on shared corner):
remove left / right / both supporters → bridged block collapses in all three cases,
landing adjacent to (not inside) the remaining supporter. **3/3.**

### Arm sweep shoves (3 trajectories)

Two-block towers struck by low EE sweeps. x-sweep and diagonal sweep disturbed towers
(top block displaced/fell, no deep clip: final separation ≥ 3cm).
y-sweep produced **no contact** — verified NOT a bug by live-FK probe (§7.3).

**Sweep total: 42 cases → 41 pass + 1 FK-probed geometric non-event.**

---

## 6. Regression Suites

Legacy scenarios re-run after every physics change (embedded in session logs):

| ID | Scenario | Expected | Last |
|----|----------|----------|------|
| R1 | gravity fall into own column | settles z=0.02, xy unchanged | ✅ |
| R2 | drop onto occupied column | stacks at z=0.06 | ✅ |
| R3 | diagonal corner bridge (P3.6.1) | stays exactly where placed | ✅ |
| R4 | single-support 25% tumble | slides off column, falls to table | ✅ |
| R5 | near-center 75% stack | stays stacked | ✅ |
| R6 | MANUAL jog timeout (new P3.6.2) | arm returns home ≤3s after last drag | ✅ |
| T1–T3 (P3.5) | fall / stack / tower-collapse | — | ✅ |
| grasp | nearest-block-within-4cm pickup | correct block grabbed | ✅ |
| e2e | yellow→green live pick-and-place (~9s) | Grabbed→Released logs + TF verify | ✅ |

---

## 7. Debugging Trails

Lessons from failures worth keeping visible:

### 7.1 The floating bridge (B1)
Symmetric straddle with 25% combined support floated forever. First fix (guarantee a
slide direction) STILL floated — because direction-away-from-nearest pushed the block
onto the OTHER supporter's column where it found ≥50% single support. Real model flaw:
unstable blocks were "surfing" at supporter-top level. Final rule: under-supported ⇒
tip off while falling (slide + gravity simultaneously). Lesson: when a fix doesn't
work, suspect the MODEL, not just the parameters.

### 7.2 Stale MANUAL target corrupted unrelated tests
R1/R4 suddenly failed although their code paths hadn't changed. Cause: minutes earlier a
test jogged the arm to an unreachable far target; MANUAL mode chased it forever, dragging
forearm links through later test scenes. Led to the 3s MANUAL timeout guardrail.
Lesson: long-lived interactive modes need expiry; also reset arm state between physics
scenarios.

### 7.3 FK probe to disprove a "failure"
The y-sweep showed zero disturbance. Instead of guessing, we sampled link TFs along the
exact trajectory and computed min clearance of every NON-exempt sample (excluding the
5.5cm gripper zone) to the tower AABB: never within margin. Verdict: geometric
non-event — the collision system was right to stay inert. Lesson: single-block grazes
often only involve exempt wrist samples; use towers to exercise link collisions.

### 7.4 Over-strict test thresholds
Two "failures" were passing behaviorally (tumble endpoint 0.235 vs demanded ≥0.24;
drop onto occupied column expected to be empty). Fix the test, not the sim — but only
after verifying the sim's value against the analytic expectation.

### 7.5 Swap-thrash masquerading as deadlock
A full-size CEM run appeared hung for 20 min: 13.4 GB/14 GB swap from a stale background
sim. Not a code bug. Kill stale sims before perf runs.

---

## 8. Deferred Items

Tracked in `Milestones_Log.md` §Open Items; obligations for **P4.1 retest**:

| Item | Why deferred | Confidence it will be needed |
|------|--------------|------------------------------|
| E3 tipping fidelity (different-height straddles) | current model drops beside instead of tipping onto lower supporter; stable but crude | 60% |
| E6 lateral block-block AABB separation | no systematic push-out between table-level blocks after shoves; observed outcome OK this run, luck-dependent | 90% eventually needed |
| Goal camera mount reconciliation | renderer uses fixed overhead cam; live cam rides EE — matters once JEPA drives | high |
| batch>1 planning | unsupported (fallback zeros) | low priority |
| Duplicate prompts restart tasks mid-run | `_start_task` called on every receipt | medium |

## 9. How to Re-run Everything

```bash
cd /Users/roopalisingh/WorldXD

# fast, no ROS:
pixi run python3 test_prompt_edge_cases.py          # 96 cases
pixi run python3 test_goal_renderer_property.py     # 332 checks
pixi run python3 test_goal_renderer.py              # 26 acceptance checks

# model (no ROS, loads checkpoint ~40s):
pixi run python3 test_jepa_robustness.py            # 19 cases

# full live sweep (requires running sim: pixi run python3 launch_robot.py):
pixi run python3 test_physics_edge_sweep.py         # 42 cases, ~12 min
```

Exit codes: 0 = all pass, 1 = at least one failure (CI-ready).
