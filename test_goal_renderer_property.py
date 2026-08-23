"""Property tests for goal_renderer (30 randomized seeds + invariants).
Run: pixi run python3 test_goal_renderer_property.py"""
import sys
import numpy as np
import cv2
sys.path.insert(0, '/Users/roopalisingh/WorldXD')
from goal_renderer import render_goal, project_block

rng = np.random.default_rng(42)
fails = []
n_cases = 0

def check(cond, name, detail=''):
    global n_cases
    n_cases += 1
    if not cond:
        fails.append(f'{name} {detail}')

COLOR_DOM = {0: ('b', 'r'), 1: ('g',), 2: ('b',), 3: ('r', 'g')}  # BGR dominant channels

for seed in range(30):
    r = np.random.default_rng(seed)
    n_blocks = r.integers(1, 5)
    blocks = [{'id': int(i),
               'pos': [float(r.uniform(0.08, 0.32)),
                       float(r.uniform(-0.25, 0.25)),
                       float(r.choice([0.02, 0.06]))]}
              for i in range(n_blocks)]
    img = render_goal(blocks)
    check(img.shape == (224, 224, 3), f'seed{seed}: shape', str(img.shape))
    check(img.dtype == np.uint8, f'seed{seed}: dtype')
    check(img.min() >= 0 and img.max() <= 255, f'seed{seed}: range')
    check(img.mean() > 10, f'seed{seed}: non-black', f'mean={img.mean():.1f}')

    # determinism
    img2 = render_goal(blocks)
    check(np.array_equal(img, img2), f'seed{seed}: deterministic')

    # every block projects somewhere in-frame with its color visible nearby
    for b in blocks:
        u, v, depth, size = project_block(b['pos'])
        if b['pos'][2] < 0.01:
            continue
        check(0 <= u < 224 and 0 <= v < 224, f'seed{seed}: block{b["id"]} in-frame',
              f'u={u} v={v}')
        x0, x1 = max(0, u - size), min(224, u + size + 1)
        y0, y1 = max(0, v - size), min(224, v + size + 1)
        patch = img[y0:y1, x0:x1].reshape(-1, 3).astype(int)
        # block colors: red(0,0,255) green(0,255,0) blue(255,0,0) yellow(0,255,255) BGR
        doms = {'red': patch[:, 2], 'green': patch[:, 1],
                'blue': patch[:, 0], 'yellow': (patch[:, 1] + patch[:, 2]) / 2}
        target = ['red', 'green', 'blue', 'yellow'][b['id']]
        best = max(doms.items(), key=lambda kv: kv[1].max() - (
            sum(v.max() for k, v in doms.items() if k != target)))
        found = any((patch[:, 2] > 180).any() and patch[:, 0].mean() < patch[:, 2].mean()
                    for _ in [0]) if False else True
        # simpler robust check: the target channel signature appears strongly
        sig = {
            'red': lambda p: ((p[:, 2] > 150) & (p[:, 0] < 120)).any(),
            'green': lambda p: ((p[:, 1] > 150) & (p[:, 2] < 120)).any(),
            'blue': lambda p: ((p[:, 0] > 150) & (p[:, 1] < 120)).any(),
            'yellow': lambda p: ((p[:, 1] > 150) & (p[:, 2] > 150) & (p[:, 0] < 120)).any(),
        }[target](patch.astype(np.uint8))
        check(sig, f'seed{seed}: block{b["id"]} ({target}) color visible',
              f'patch mean={patch.mean(axis=0)}')

# degenerate inputs must not crash
weird = [
    [],                                             # empty scene
    [{'id': 0, 'pos': [-1.0, -5.0, -2.0]}],         # far out of frame
    [{'id': 1, 'pos': [0.15, 0.10, 0.005]}],        # behind camera plane
    [{'id': 2, 'pos': [100.0, 100.0, 50.0]}],       # absurd distance
]
for i, w in enumerate(weird):
    try:
        img = render_goal(w)
        check(img.shape == (224, 224, 3), f'weird{i}: shape ok')
    except Exception as e:
        check(False, f'weird{i}: crashed', str(e)[:60])

print(f'TOTAL={n_cases} FAIL={len(fails)}')
for f in fails:
    print('FAIL:', f)
cv2.imwrite('captures/goal_property_sample.png', render_goal(
    [{'id': i, 'pos': p} for i, p in enumerate(
        [[0.15, 0.1, 0.02], [0.20, 0.1, 0.02], [0.15, -0.1, 0.06], [0.20, -0.1, 0.02]])]))
sys.exit(0 if not fails else 1)
