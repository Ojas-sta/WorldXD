"""Live physics edge-case sweep against the running RViz2 sim.
Run: pixi run python3 test_physics_edge_sweep.py
Every case computes the EXPECTED outcome analytically from the same footprint
rule the sim uses, then checks the sim's settled state matches."""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

rclpy.init()
n = Node('sweep')
pub = n.create_publisher(PointStamped, '/block_move', 10)
snap = {}
n.create_subscription(MarkerArray, 'workspace_blocks',
    lambda m: snap.update({k.id:(round(k.pose.position.x,3),round(k.pose.position.y,3),round(k.pose.position.z,3)) for k in m.markers if k.type==1}), 10)
rp = n.create_publisher(String, 'user_prompt', 10)

H = 0.04
HALF = 0.02
def sample(sec=1.2):
    t0=time.time()
    while time.time()-t0<sec: rclpy.spin_once(n, timeout_sec=0.05)
    return dict(snap)

def drag(bid,x,y,z,msgs=8):
    p = PointStamped(); p.header.frame_id=f'block_{bid}'
    p.point.x,p.point.y,p.point.z = x,y,z
    for _ in range(msgs): pub.publish(p); time.sleep(0.04)

def settle(bid, want_z, tol=0.006, max_wait=5.0):
    """poll until block bid settles at want_z or timeout; return last pos"""
    t0=time.time(); last=None
    while time.time()-t0<max_wait:
        rclpy.spin_once(n, timeout_sec=0.05)
        last = snap.get(bid)
        if last and abs(last[2]-want_z)<tol:
            time.sleep(0.3); return sample(0.2).get(bid, last)
    return sample(0.3).get(bid, last)

def reset():
    m = String(); m.data='reset'
    for _ in range(6): rp.publish(m); time.sleep(0.04)
    time.sleep(1.2)

results = []
def record(name, ok, detail):
    results.append((name, ok, detail))
    print(f'{"PASS" if ok else "FAIL"} | {name} | {detail}')

def expected_stable(dx, dy):
    fx = max(H - abs(dx), 0.0); fy = max(H - abs(dy), 0.0)
    return (fx * fy) / (H * H) >= 0.5

YEL = [0.20, -0.10]   # yellow home

# --- GRID: 25 placements over a single supporter ---
print('--- GRID: 5x5 offsets over yellow ---')
for gx, dx in enumerate([-0.036, -0.018, 0.0, 0.018, 0.036]):
    for gy, dy in enumerate([-0.036, -0.018, 0.0, 0.018, 0.036]):
        reset()
        px, py = YEL[0]+dx, YEL[1]+dy
        drag(2, px, py, 0.07)
        exp_stable = expected_stable(dx, dy)
        want = 0.06 if exp_stable else 0.02
        b = settle(2, want)
        ok = b is not None and abs(b[2]-want) < 0.008
        record(f'grid dx={dx:+.3f} dy={dy:+.3f} expect={"stack" if exp_stable else "fall"}',
               ok, f'got z={b[2] if b else None}')

# --- DIAGONAL sweep ---
print('--- DIAGONAL sweep ---')
for d in [-0.034, -0.025, -0.015, 0.0, 0.015, 0.025, 0.034]:
    reset()
    drag(2, YEL[0]+d, YEL[1]+d, 0.07)
    exp_stable = expected_stable(d, d)
    want = 0.06 if exp_stable else 0.02
    b = settle(2, want)
    ok = b is not None and abs(b[2]-want) < 0.008
    record(f'diag d={d:+.3f} expect={"stack" if exp_stable else "fall"}', ok,
           f'got z={b[2] if b else None}')

# --- DROP HEIGHTS ---
print('--- DROP HEIGHTS ---')
for h in [0.05, 0.09, 0.14, 0.20]:
    reset()
    drag(2, YEL[0], YEL[1], h)
    b = settle(2, 0.02)
    ok = b is not None and abs(b[2]-0.02) < 0.005
    record(f'drop from z={h:.2f} onto empty column', ok, f'got {b}')

# --- SUPPORT REMOVAL from bridge (3 permutations) ---
print('--- BRIDGE SUPPORT REMOVAL ---')
BR_A = [0.165, -0.10]; BR_B = [0.235, -0.10]
for which in ['left', 'right', 'both']:
    reset()
    drag(0, BR_A[0], BR_A[1], 0.02); drag(3, BR_B[0], BR_B[1], 0.02); time.sleep(1.2)
    drag(2, 0.20, -0.10, 0.07); time.sleep(2.0)   # bridge it (50% total)
    mid = sample().get(2)
    bridged = mid and abs(mid[2]-0.06) < 0.008
    if which in ('left', 'both'):
        drag(0, 0.30, 0.18, 0.02)
    if which in ('right', 'both'):
        drag(3, 0.12, 0.18, 0.02)
    time.sleep(3.5)
    b = sample().get(2)
    fell = b is not None and b[2] < mid[2] - 0.03
    record(f'bridge remove {which}: fell={fell}', bridged and fell, f'mid={mid} after={b}')

# --- ARM SWEEP SHOVE (3 trajectories) ---
print('--- ARM SWEEP SHOVE ---')
for name, path in [
    ('low sweep y-', [(0.15, -0.24, 0.04), (0.15, -0.02, 0.04)]),
    ('low sweep x+', [(0.08, -0.10, 0.04), (0.28, -0.10, 0.04)]),
    ('diagonal sweep', [(0.10, -0.18, 0.05), (0.26, 0.02, 0.05)]),
]:
    reset()
    # park blue near center table where sweeps cross
    drag(2, 0.16, -0.11, 0.02); time.sleep(1)
    before = sample()[2]
    steps = 30
    for i in range(steps):
        t = i/(steps-1)
        x = path[0][0] + (path[1][0]-path[0][0])*t
        y = path[0][1] + (path[1][1]-path[0][1])*t
        z = path[0][2]
        p = PointStamped(); p.header.frame_id='base_link'
        p.point.x, p.point.y, p.point.z = x, y, z
        ee_pub = n.create_publisher(PointStamped, '/ee_target', 10)
        for _ in range(2): ee_pub.publish(p); time.sleep(0.03)
    time.sleep(4.5)   # allow MANUAL timeout + retreat
    b = sample()[2]
    moved = abs(b[0]-before[0])>0.008 or abs(b[1]-before[1])>0.008
    record(f'arm sweep "{name}" disturbs block', moved, f'before={before} after={b}')

print('\n===== SWEEP SUMMARY =====')
p = sum(1 for _,ok,_ in results if ok)
print(f'TOTAL={len(results)} PASS={p} FAIL={len(results)-p}')
sys.exit(0 if p == len(results) else 1)
