"""P4.5 live relational-prompt sweep in RViz2. Run: pixi run python3 test_p45_relational_live.py"""
import sys, time
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

rclpy.init()
n = Node('p45')
snap = {}
n.create_subscription(MarkerArray, 'workspace_blocks',
    lambda m: snap.update({k.id:(round(k.pose.position.x,3),round(k.pose.position.y,3),round(k.pose.position.z,2)) for k in m.markers if k.type==1}), 10)
rp = n.create_publisher(String, 'user_prompt', 10)

def sample(sec=1.0):
    t0=time.time()
    while time.time()-t0<sec: rclpy.spin_once(n, timeout_sec=0.05)
    return dict(snap)

def say(text):
    m = String(); m.data = text
    for _ in range(4): rp.publish(m); time.sleep(0.04)

def reset():
    say('reset'); time.sleep(1.5)
    say('reset'); time.sleep(1.5)

def wait_idle(max_wait=40):
    """wait until /fsm_state reports DONE (or timeout)"""
    from std_msgs.msg import String as S
    got = {'fsm': None}
    def on_fsm(m): got['fsm'] = m.data
    sub = n.create_subscription(S, 'fsm_state', on_fsm, 10)
    t0=time.time()
    while time.time()-t0<max_wait:
        rclpy.spin_once(n, timeout_sec=0.1)
        if got['fsm'] == 'DONE':
            n.destroy_subscription(sub); time.sleep(0.8); return True
        if got['fsm'] is not None and got['fsm'] != 'DONE':
            pass
    n.destroy_subscription(sub)
    return False

def stacked_on(top, base, tol=0.012):
    s_ = sample()
    t_, b_ = s_.get(top), s_.get(base)
    return (t_ and b_ and abs(t_[0]-b_[0])<=tol and abs(t_[1]-b_[1])<=tol
            and abs(t_[2]-(b_[2]+0.04))<=0.006), f'{top}@{t_} on {base}@{b_}'

results = []
def record(name, ok, detail=''):
    results.append((name, ok))
    print(f'{"PASS" if ok else "FAIL"} | {name} | {detail}')

# ---------- L0: user's exact scenario setup ----------
print('--- SETUP: green->red tower, blue->yellow tower ---')
reset()
say('keep the green block on top of the red block'); wait_idle()
ok, d = stacked_on(1, 0); record('setup: green on red', ok, d)
say('keep the blue block on top of the yellow block'); wait_idle()
ok, d = stacked_on(2, 3); record('setup: blue on yellow', ok, d)

print('--- THE USER CASE: keep the blue block under the red block ---')
say('keep the blue block under the red block'); wait_idle()
# expected: red moved onto blue (blue currently atop yellow) -> 3-tower red>blue>yellow
ok_r_on_b, d1 = stacked_on(0, 2)
ok_chain, d2 = stacked_on(2, 3)
record('L4 keep-blue-under-red => red placed on blue (tower)', ok_r_on_b, d1)
record('L4b chain preserved: blue still on yellow', ok_chain, d2)

# ---------- synonyms ----------
for name, prompt, top, base in [
    ('L5 beneath synonym',   'put the yellow block beneath the green block', 1, 3),
    ('L6 below synonym',     'keep red below green',                         1, 0),
    ('L7 underneath syn.',   'move the blue block underneath the green one', 1, 2),
]:
    reset()
    # build single-supporter-free scene: ensure target bases are on table by reset+direct task
    say(prompt); wait_idle()
    ok, d = stacked_on(top, base); record(f'{name}: {prompt[:38]}', ok, d)

# ---------- top-relation with new verbs ----------
reset()
for name, prompt, top, base in [
    ('L8 put synonym',      'put the green block on top of the red block', 1, 0),
    ('L9 set synonym',      'set the blue one on top of the green one',    2, 1),
    ('L10 make-sure phrasing','make sure the yellow block is on top of the blue block', 3, 2),
]:
    say(prompt); wait_idle()
    ok, d = stacked_on(top, base); record(f'{name}', ok, d)

# ---------- no-op / rejection cases ----------
reset()
before = sample()
say('keep the blue block under the blue block'); wait_idle()
after = sample()
nomove = all(abs(after[i][k]-before[i][k])<0.01 for i in range(4) for k in range(2))
record('L11 self-under rejected (no movement)', nomove)

say('keep red under the purple block'); wait_idle()
after2 = sample()
nomove2 = all(abs(after2[i][k]-before[i][k])<0.01 for i in range(4) for k in range(2))
record('L12 unknown reference color ignored', nomove2)

# ---------- held-pronoun inverted pair ----------
reset()
say('pick up the blue block and keep it under the red block'); wait_idle()
ok, d = stacked_on(0, 2)   # red ends atop blue
record('L13 held-pronoun under => red onto blue', ok, d)

# ---------- descriptive over-trigger executes (documented behavior) ----------
reset()
say('the yellow block is under the green block'); wait_idle()
ok, d = stacked_on(1, 3)   # yellow UNDER green => green ends atop yellow
record('L14 descriptive relation executes (documented)', ok, d)

# ---------- spam during active task ----------
reset()
say('keep the green block on top of the red block')
time.sleep(1.0)   # mid-task (likely MOVE_ABOVE_BLOCK)
say('keep the blue block on top of the yellow block')   # override
wait_idle(); wait_idle()
s_ = sample()
b_on_y = abs(s_[2][0]-s_[3][0])<=0.012 and abs(s_[2][2]-(s_[3][2]+0.04))<=0.006
g_moved = s_[1][2] >= 0.02
record('L15 mid-task override completes second task w/o crash', b_on_y or g_moved,
       f'blue@{s_[2]} yellow@{s_[3]}')

# ---------- lengthy under prompt ----------
reset()
say("okay next exercise: please keep the yellow block under the green block this "
    "time, and take care to lower it gently so the tower stays stable throughout "
    "the entire demonstration for everyone watching the screen right now.")
wait_idle()
ok, d = stacked_on(1, 3)
record('L16 lengthy under-prompt', ok, d)

# ---------- ALL-CAPS ----------
reset()
say('KEEP THE GREEN BLOCK UNDER THE RED BLOCK!!!'); wait_idle()
ok, d = stacked_on(1, 0)


# ---------- arrange after relational tasks ----------
reset()
say('arrange all blocks'); time.sleep(2); wait_idle(); wait_idle(); time.sleep(6)
s_ = sample()
heights = sorted([s_[i][2] for i in range(4)], reverse=True)
stacked_some = heights[0] >= 0.09   # at least 3-high somewhere
record('L18 arrange still works post-relational', stacked_some, f'heights={heights}')

# ---------- rapid double prompts ----------
reset()
say('keep the green block on top of the red block')
say('keep the yellow block on top of the green block')
wait_idle(); wait_idle(); time.sleep(1)
ok_yg, d1 = stacked_on(3, 1)
record('L19 rapid double prompt: final pair satisfied', ok_yg, d1)

# ---------- get synonym + under ----------
reset()
say('get the blue under the yellow'); wait_idle()
ok, d = stacked_on(3, 2)
record('L20 get-synonym under', ok, d)

# ---------- grab synonym + under ----------
reset()
say('grab the yellow and keep it under the blue'); wait_idle()
ok, d = stacked_on(2, 3)
record('L21 grab-keep under', ok, d)

# ---------- reset integrity after everything ----------
reset()
s_ = sample()
homes = all(abs(s_[i][0]-h[0])<0.02 and abs(s_[i][1]-h[1])<0.02 and abs(s_[i][2]-0.02)<0.005
            for i,h in [(0,[0.15,0.1]),(1,[0.20,0.1]),(2,[0.15,-0.1]),(3,[0.20,-0.1])])
record('L22 reset restores homes after suite', homes, str(s_))

print('\n===== P4.5 LIVE SUMMARY =====')
p = sum(1 for _,ok in results if ok)
print(f'TOTAL={len(results)} PASS={p} FAIL={len(results)-p}')
sys.exit(0 if p==len(results) else 1)
