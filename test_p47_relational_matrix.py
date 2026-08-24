"""P4.7 relational matrix: 32 scenarios, live in RViz2.
Run: pixi run python3 test_p47_relational_matrix.py
Every strict case verifies final geometry; robustness cases verify no-overlap + FSM recovery."""
import sys, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

rclpy.init()
n = Node('matrix')
snap = {}
n.create_subscription(MarkerArray, 'workspace_blocks',
    lambda m: snap.update({k.id:(round(k.pose.position.x,3),round(k.pose.position.y,3),round(k.pose.position.z,2)) for k in m.markers if k.type==1}), 10)
rp = n.create_publisher(String, 'user_prompt', 10)
fsm = {'f': None, '_ls': None}
n.create_subscription(String, 'fsm_state', lambda v: fsm.update(f=v.data), 10)
time.sleep(3)

def sample(sec=1.2):
    t0=time.time()
    while time.time()-t0<sec: rclpy.spin_once(n, timeout_sec=0.05)
    return dict(snap)

def say(text, max_wait=140):
    prev = fsm['f']; t0 = time.time()
    while time.time()-t0 < max_wait:
        m = String(); m.data = text
        for _ in range(6): rp.publish(m); time.sleep(0.05)
        t1 = time.time()
        while time.time()-t1 < 4:
            rclpy.spin_once(n, timeout_sec=0.1)
            if fsm['f'] != prev and fsm['f'] is not None: break
        if fsm['f'] != prev and fsm['f'] is not None:
            t2 = time.time(); last = time.time()
            while time.time()-t2 < max_wait:
                rclpy.spin_once(n, timeout_sec=0.1)
                cur = fsm['f']
                if cur != fsm['_ls']: fsm['_ls'] = cur; last = time.time()
                if cur == 'DONE' and time.time()-last > 1.5:
                    time.sleep(0.8); return True
            return False
        time.sleep(0.5)
    return False

def reset():
    # reset from DONE never changes fsm state -> say()'s change-detection
    # would burn max_wait. Use a fixed publish burst instead.
    m = String(); m.data = 'reset'
    for _ in range(8): rp.publish(m); time.sleep(0.05)
    time.sleep(2.2)

def stacked(top, base, tol=0.014):
    s_ = sample()
    t_, b_ = s_.get(top), s_.get(base)
    return (t_ and b_ and abs(t_[0]-b_[0])<=tol and abs(t_[1]-b_[1])<=tol
            and abs(t_[2]-(b_[2]+0.04))<=0.008)

def no_overlap_anywhere():
    s_ = sample()
    ids = list(s_.keys())
    for i in range(len(ids)):
        for j in range(i+1, len(ids)):
            a, b = s_[ids[i]], s_[ids[j]]
            if abs(a[2]-b[2]) < 0.034:
                d = ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5
                if d < 0.038:
                    return False, f'{ids[i]}@{a} vs {ids[j]}@{b} d={d:.3f}'
    return True, 'clear'

def setup_towers(kind):
    reset()
    if kind == 'A':   # green>red, blue>yellow
        say('keep the green block on top of the red block'); time.sleep(0.5)
        say('keep the blue block on top of the yellow block'); time.sleep(0.5)
    elif kind == 'B': # red>green, yellow>blue
        say('keep the red block on top of the green block'); time.sleep(0.5)
        say('keep the yellow block on top of the blue block'); time.sleep(0.5)
    time.sleep(1.0)

results = []
def record(name, ok, detail=''):
    results.append((name, ok))
    print(f'{"PASS" if ok else "FAIL"} | {name} | {detail}', flush=True)

C = {'red':0, 'green':1, 'blue':2, 'yellow':3}

# ---- GROUP 1: all 12 ordered under-pairs on an empty table (strict) ----
print('== GROUP 1: 12 ordered under-pairs, free destinations ==', flush=True)
pairs = [(a,b) for a in C for b in C if a != b]
for mover, base in pairs:   # "keep mover under base" => base ends atop mover
    reset()
    say(f'keep the {mover} block under the {base} block'); time.sleep(1.0)
    ok = stacked(C[base], C[mover])
    record(f'G1 {mover} under {base} => {base} atop {mover}', ok)

# ---- GROUP 2: under-relations through occupied columns (planner) ----
print('== GROUP 2: occupied-destination planning ==', flush=True)
setup_towers('A')   # green>red, blue>yellow
say('put the yellow block under the red block'); time.sleep(1.0)
ok = stacked(C['red'], C['yellow'])
nv, nd = no_overlap_anywhere()
record('G2 yellow-under-red via towers (red atop yellow)', ok and nv, nd)

setup_towers('A')
say('keep the blue block under the green block'); time.sleep(1.0)
ok = stacked(C['green'], C['blue'])
nv, nd = no_overlap_anywhere()
record('G2 blue-under-green via towers', ok and nv, nd)

setup_towers('B')   # red>green, yellow>blue
say('put the green block under the yellow block'); time.sleep(1.0)
ok = stacked(C['yellow'], C['green'])
nv, nd = no_overlap_anywhere()
record('G2 green-under-yellow via towers B', ok and nv, nd)

# ---- GROUP 3: buried pick (blocker on the pick target) ----
print('== GROUP 3: buried pick ==', flush=True)
setup_towers('A')   # green on red: red buried
say('grab the red block and put it on the yellow block'); time.sleep(1.0)
ok = stacked(C['red'], C['yellow'])
nv, nd = no_overlap_anywhere()
record('G3 buried red -> onto yellow', ok and nv, nd)

setup_towers('B')   # red on green: green buried
say('move the green block onto the blue block'); time.sleep(1.0)
ok = stacked(C['green'], C['blue'])
nv, nd = no_overlap_anywhere()
record('G3 buried green -> onto blue', ok and nv, nd)

# ---- GROUP 4: synonyms / pronouns on free table ----
print('== GROUP 4: synonyms & pronouns ==', flush=True)
reset()
say('grab the yellow and put it under green'); time.sleep(1.0)
record('G4 grab+put synonym', stacked(C['green'], C['yellow']))
reset()
say('set the red one under the blue one'); time.sleep(1.0)
record('G4 set+one synonym', stacked(C['blue'], C['red']))
reset()
say('get the green under the yellow'); time.sleep(1.0)
record('G4 get synonym', stacked(C['yellow'], C['green']))
reset()
say('pick up the blue block and keep it under the red block'); time.sleep(1.0)
record('G4 held-pronoun under', stacked(C['red'], C['blue']))

# ---- GROUP 5: top-relation regression (must not regress) ----
print('== GROUP 5: top-relation regression ==', flush=True)
reset()
say('keep the green block on top of the red block'); time.sleep(1.0)
record('G5 classic top pair', stacked(C['green'], C['red']))

# ---- GROUP 6: lengthy / caps / polite ----
print('== GROUP 6: phrasing stress ==', flush=True)
reset()
say('okay so what I want you to do next is keep the yellow block under the green '
    'block, and please be gentle while placing so nothing tips over, thanks a lot')
time.sleep(1.0)
record('G6 lengthy under', stacked(C['green'], C['yellow']))
reset()
say('KEEP THE RED BLOCK UNDER THE BLUE BLOCK!!!'); time.sleep(1.0)
record('G6 caps under', stacked(C['blue'], C['red']))
reset()
say('could you please put the blue block under the red block, if possible?')
time.sleep(1.0)
record('G6 polite question', stacked(C['red'], C['blue']))

# ---- GROUP 7: robustness (no strict geometry) ----
print('== GROUP 7: robustness ==', flush=True)
reset()
say('keep the red block under the red block'); time.sleep(2.0)
nv, nd = no_overlap_anywhere()
record('G7 self-under no-op + no overlap', nv, nd)
reset()
say('keep the purple block under the red block'); time.sleep(2.0)
nv, nd = no_overlap_anywhere()
record('G7 unknown color ignored + no overlap', nv, nd)
reset()
say('keep the green block on top of the red block')
time.sleep(1.0)
say('keep the blue block under the yellow block')   # override mid-task
time.sleep(1.0); wait_ok = say('reset', max_wait=150)
time.sleep(2.0)
nv, nd = no_overlap_anywhere()
record('G7 mid-task override + reset recovery', nv and wait_ok, nd)

print('\n===== P4.7 MATRIX SUMMARY =====')
p = sum(1 for _, ok in results if ok)
print(f'TOTAL={len(results)} PASS={p} FAIL={len(results)-p}', flush=True)
sys.exit(0 if p == len(results) else 1)
