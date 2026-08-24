"""Verification for goal_renderer.py (P4 JEPA goal-image renderer).

Renders the home layout and a yellow-over-green stack using the exact
workspace_env camera conventions and writes inspection PNGs to captures/.

Run: pixi run python3 test_goal_renderer.py
Expected: one PASS line per test, exit 0; PNGs in captures/.
"""
import os
import sys

import cv2
import numpy as np

from goal_renderer import (BLOCK_HOME, home_blocks, project_block,
                           render_goal, stacked_config)

FAILURES = []


def check(test, name, cond, detail=''):
    status = 'PASS' if cond else 'FAIL'
    line = f'[{status}] {test}: {name}'
    if detail:
        line += f' ({detail})'
    print(line)
    if not cond:
        FAILURES.append(line)
    return cond


# After the 0.3/0.7 ArUco blend the grayscale marker adds equally to all
# channels, so each block colour keeps its dominance over the OTHER channels
# (yellow's G and R are equal, so yellow is tested as "G and R both beat B").
def dominant_ok(img, u, v, bid):
    win = slice(max(0, v - 2), v + 3), slice(max(0, u - 2), u + 3)
    b = img[win[0], win[1], 0].astype(int)
    g = img[win[0], win[1], 1].astype(int)
    r = img[win[0], win[1], 2].astype(int)
    if bid == 0:                                   # red: R beats G and B
        return bool(np.any((r > g) & (r > b)))
    if bid == 1:                                   # green: G beats R and B
        return bool(np.any((g > r) & (g > b)))
    if bid == 2:                                   # blue: B beats R and G
        return bool(np.any((b > r) & (b > g)))
    return bool(np.any((g > b) & (r > b)))         # yellow: G,R beat B


def test_a_home_layout():
    blocks = home_blocks()
    img = render_goal(blocks)

    check('A', 'output shape', img.shape == (224, 224, 3), str(img.shape))
    check('A', 'dtype uint8', img.dtype == np.uint8, str(img.dtype))
    check('A', 'value range', int(img.min()) >= 0 and int(img.max()) <= 255,
          f'min={img.min()} max={img.max()}')
    mean = float(img.mean())
    check('A', 'non-black (mean>10)', mean > 10, f'mean={mean:.1f}')

    for blk in blocks:
        bid = blk['id']
        u, v, z, size = project_block(blk['pos'])
        in_frame = check('A', f'block {bid} projects in-frame',
                         z >= 0.01 and 0 <= u < 224 and 0 <= v < 224,
                         f'u={u} v={v} depth={z:.3f} size={size}')
        if not in_frame:
            continue
        # The pixel at the projected centre must carry the block colour even
        # after the ArUco blend; scan +-2 px to absorb rounding.
        check('A', f'block {bid} coloured at projection',
              dominant_ok(img, u, v, bid), f'at (u={u}, v={v})')


def test_b_stacked_alignment():
    base_id, mover_id = 1, 3  # green base, yellow on top
    cfg = stacked_config(base_id, [mover_id])

    ok_struct = check('B', 'config length', len(cfg) == 2, str(len(cfg)))
    ok_struct &= check('B', 'base stays at home',
                       cfg[0]['pos'] == list(BLOCK_HOME[base_id]), str(cfg[0]['pos']))
    ok_struct &= check('B', 'mover shares stack xy',
                       cfg[1]['pos'][:2] == cfg[0]['pos'][:2], str(cfg[1]['pos'][:2]))
    ok_struct &= check('B', 'mover z = +0.04',
                       abs(cfg[1]['pos'][2] - (cfg[0]['pos'][2] + 0.04)) < 1e-9,
                       str(cfg[1]['pos'][2]))

    try:
        img = render_goal(cfg)
        rendered = True
    except Exception as e:
        rendered = check('B', 'render with overlapping xy does not crash', False, repr(e))
    if rendered:
        check('B', 'rendered shape', img.shape == (224, 224, 3))
        check('B', 'overlap render non-black', float(img.mean()) > 10,
              f'mean={img.mean():.1f}')

    ub, vb, zb, sb = project_block(cfg[0]['pos'])
    um, vm, zm, sm = project_block(cfg[1]['pos'])
    # Perspective: identical xy but +0.04 m depth still shifts u,v by a pixel
    # or two under focal_length=200; allow that much jitter.
    check('B', 'mover aligns with base in u', abs(um - ub) <= 3, f'du={abs(um - ub)}')
    check('B', 'mover aligns with base in v', abs(vm - vb) <= 3, f'dv={abs(vm - vb)}')
    check('B', 'mover size <= base size', sm <= sb, f'sm={sm} sb={sb}')
    check('B', 'both blocks visible', min(zb, zm) >= 0.01,
          f'zb={zb:.3f} zm={zm:.3f}')
    check('B', 'stack pixels present at alignment point',
          dominant_ok(render_goal(cfg), ub, vb, mover_id),
          f'yellow near (u={ub}, v={vb})')


def test_c_save_pngs():
    os.makedirs('captures', exist_ok=True)
    paths = {
        'home': os.path.join('captures', 'goal_home.png'),
        'stacked': os.path.join('captures', 'goal_stacked.png'),
    }
    cv2.imwrite(paths['home'], render_goal(home_blocks()))
    cv2.imwrite(paths['stacked'], render_goal(stacked_config(1, [3])))
    for label, path in paths.items():
        ok = os.path.isfile(path) and os.path.getsize(path) > 0
        check('C', f'saved {path}', ok, f'{os.path.getsize(path)} bytes' if ok else 'missing')


def main():
    print('=' * 60)
    print('goal_renderer verification')
    print('=' * 60)
    test_a_home_layout()
    test_b_stacked_alignment()
    test_c_save_pngs()
    print('-' * 60)
    if FAILURES:
        print(f'{len(FAILURES)} check(s) FAILED')
        sys.exit(1)
    print('ALL TESTS PASSED')


if __name__ == '__main__':
    main()
