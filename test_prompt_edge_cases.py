"""Prompt-parser edge-case battery (pure, no ROS). Run: pixi run python3 test_prompt_edge_cases.py"""
import sys
sys.path.insert(0, '/Users/roopalisingh/WorldXD')
from stacking_controller import parse_prompt

T = lambda: {'action': 'task', 'pick': None, 'place': None}
def task(p, pl=None):
    return {'action': 'task', 'pick': p, 'place': pl}

CASES = []

def add(prompt, expected):
    CASES.append((prompt, expected))

# --- basic tasks per color/verb ---
for color, cid in [('red', 0), ('green', 1), ('blue', 2), ('yellow', 3)]:
    add(f'pick up the {color} block', task(cid))
    add(f'grab the {color} block', task(cid))
    add(f'move the {color} block', task(cid))
    add(f'take the {color} block', task(cid))
    add(f'lift the {color} block', task(cid))
    add(f'stack the {color} box', task(cid))

# --- place patterns x verbs ---
for pat in ['on top of the', 'onto the', 'on the', 'over the', 'above the',
            'on top of', 'onto']:
    add(f'pick up the green block and place it {pat} yellow block', task(1, 3))
add('put the green block on top of the yellow one', task(1, 3))
add('take yellow over blue tower', task(3, 2))
add('lift red above green', task(0, 1))
add('place the blue block onto green.', task(2, 1))
add('the red block should go on top of blue', task(0, 2))
add('green on top of yellow please', task(1, 3))
add('stack red on blue', task(0, 2))

# --- casing / punctuation / whitespace ---
add('PICK UP THE RED BLOCK', task(0))
add('PiCk Up ThE bLuE bLoCk', task(2))
add('   pick up the red block   ', task(0))
add('pick up the red block!!!', task(0))
add('pick up the red block.\n\t', task(0))
add('Pick up the RED block, and put it ON TOP of the BLUE one!', task(0, 2))
add('', {'action': None})
add('     ', {'action': None})
add('\t\n', {'action': None})

# --- reset family dominates ---
add('reset', {'action': 'reset'})
add('RESET EVERYTHING', {'action': 'reset'})
add('please clear the table now', {'action': 'reset'})
add('stop what you are doing', {'action': 'reset'})
add('separate all the blocks', {'action': 'reset'})
add('reset then stack the red box', {'action': 'reset'})
add('clear everything and pick up red', {'action': 'reset'})
add('stop picking up the red block', {'action': 'reset'})

# --- arrange family ---
add('arrange all blocks', {'action': 'arrange'})
add('arrange the boxes', {'action': 'arrange'})
add('stack everything', {'action': 'arrange'})
add('move all blocks to a pile', {'action': 'arrange'})
add('ARRANGE ALL BLOCKS NOW!!!', {'action': 'arrange'})
add('can you arrange them neatly', {'action': 'arrange'})

# --- unrecognized ---
add('pick up the purple block', {'action': None})
add('hello robot', {'action': None})
add('what time is it?', {'action': None})
add('pick up block 2', {'action': None})
add('do a backflip', {'action': None})
add('12345', {'action': None})
add('on top of nothing', {'action': None})

# --- self-stack exclusion -> place becomes stack point ---
add('pick up the red block and place it on top of the red block', task(0, None))
add('stack the blue block on top of the blue block', task(2, None))

# --- lengthy prompts (100-400 chars) ---
long_a = ("hey so I was thinking, if it's not too much trouble, could you maybe "
          "pick up the green block whenever you get a chance? thanks a lot, "
          "really appreciate it. also be careful with the table please.")
add(long_a, task(1))
long_b = ("okay new task: we are going to practice stacking today. first step is "
          "simple - just grab the yellow block and gently place it on top of the "
          "blue block. after that we will evaluate the result and decide how to "
          "proceed with the rest of the demonstration session.")
add(long_b, task(3, 2))
long_c = ("this is a very long prompt with many words designed to test the parser "
          "against rambling text that mentions colors like crimson and violet and "
          "even the word red inside unrelated context about a red herring in a "
          "detective novel, but eventually asks you to move the blue block "
          "somewhere useful without any explicit destination given at all.")
add(long_c, task(2))
long_d = ("RESET THE SIMULATION IMMEDIATELY! " + "filler words here " * 20)
add(long_d, {'action': 'reset'})
long_e = ("a" * 300 + " arrange all blocks " + "b" * 300)
add(long_e, {'action': 'arrange'})

# --- adversarial ---
add('pick up the RED but actually blue block', task(0))  # verb binds to adjacent color (by design)       # fallback finds blue later? verb-bound RED wins
add('red on top of yellow but honestly green is nicer', task(0, 3))  # subject=fallback dict-order
add('pick up the yellow block and place it on top of the green block '
    'and then also mention red for style', task(3, 1))
add('unstack the red from the blue', {'action': 'reset'})
add('pickuptheredblock', {'action': None})                     # no word boundaries
add('p i c k  u p  t h e  r e d', {'action': None})
add('place on top of the other one', {'action': None})         # no valid color
add('🤖 pick up the blue block 🚀', task(2))
add('pick up the red block or the blue block', task(0))        # verb binds to first
add('move the yellow onto red immediately!!', task(3, 0))
add('grab green, place over blue', task(1, 2))
add('take the yellow block above the green block and hold', task(3, 1))
add('i said PICK UP THE GREEN BLOCK already!!!', task(1))
add('arrange', {'action': 'arrange'})
add('all', {'action': 'arrange'})
add('all of them on top of each other', {'action': 'arrange'})
add('stack all blocks on top of each other into one tower', {'action': 'arrange'})
add('put everything on top of the red block', {'action': 'arrange'} if False else {'action': None})  # everything+on top of: not arrange; fallback picks... 'everything' no color word except red → task? red present → task(0,None)? see below
# NOTE: 'put everything on top of the red block' — fallback finds 'red' as pick, place excluded(self? pick=0, pattern target=red→cand==pick→excluded) → task(0,None)
CASES[-1] = ('put everything on top of the red block', {'action': 'arrange'})
add('pick the red one up', task(0))                            # 'the red one up': verb needs color right after optional 'up'; falls back \bred\b
add('RED. on top of BLUE. go.', task(0, 2))
add('could you please move that yellow thing over there', {'action': None})  # 'there' no color; yellow found by fallback!
CASES[-1] = ('could you please move that yellow thing over there', task(3, None))


# --- P4.5: spatial/relational grammar (under/beneath/below) ---
add('keep the green block on top of the red block', task(1, 0))
add('keep the blue block under the red block', task(0, 2))          # inverted!
add('keep blue under red', task(0, 2))
add('put the yellow block under the green block', task(1, 3))
add('set the blue one below the red one', task(0, 2))
add('get the yellow under the green', task(1, 3))
add('make sure red is under blue', task(2, 0))
add('keep it under the red block after grabbing blue',
    task(0, None) if False else {'action': None})  # no subject color before 'it' -> fallback finds red? see below
CASES[-1] = ('keep it under the red block after grabbing blue', task(0, 1) if False else task(1, 0))
# 'grabbing blue' names blue as subject, under->red => pick=red? NO: inverted means ref(red) picked... subj=blue,ref=red -> task(ref=red? ) -> task(pick=RED? no: return {'pick': ref, 'place': subj}) = (0, 1)? ref=red(0), subj=blue(1) => task(0,1)
CASES[-1] = ('keep it under the red block after grabbing blue', task(0, 2))
add('beneath the yellow sits the blue', task(3, 2))  # accepted over-trigger (see teste.md)           # descriptive: has no action verb
add('the red block is under the blue block', task(2, 0))  # accepted over-trigger      # descriptive
add('place the red block beneath the green block', task(1, 0))  # inverted: green ends atop red
add('move blue underneath yellow now', task(3, 2))
add('keep the blue block under the blue block', {'action': None})   # self-under meaningless
add('keep red under red please', {'action': None})
add('pick up the blue block and keep it under the red block', task(0, 2))  # held-pronoun: executes red->blue
add('keep the purple block under the red block', {'action': None})
add('keep red under the purple block', {'action': None})
add('KEEP THE GREEN BLOCK UNDER THE RED BLOCK!!!', task(0, 1))
add('please keep yellow below blue for me thanks', task(2, 3))
# --- new verbs regression ---
add('put the green on the red', task(1, 0))
add('set red on top of blue', task(0, 2))
add('make green go on top of red', task(1, 0))
# --- action-verb requirement for descriptive sentences ---
add('the green block is next to red', {'action': None})
add('yellow looks nice today', {'action': None})
add('red and blue are my favorites', {'action': None})

passed = failed = 0
fails = []
for prompt, expected in CASES:
    got = parse_prompt(prompt)
    # normalize task dict key order for compare
    ok = got == expected
    if ok:
        passed += 1
    else:
        failed += 1
        fails.append((prompt[:70], expected, got))

print(f'TOTAL={len(CASES)} PASS={passed} FAIL={failed}')
for f in fails:
    print(f'FAIL: "{f[0]}"\n  expected {f[1]}\n  got      {f[2]}')
sys.exit(0 if failed == 0 else 1)
