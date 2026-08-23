"""JEPA get_action robustness battery. Run: pixi run python3 test_jepa_robustness.py
Loads the real checkpoint once with tiny CEM params for speed."""
import sys, time
import numpy as np
import torch
import torchvision.transforms as transforms

sys.path.insert(0, '/Users/roopalisingh/WorldXD')
from jepa_model import JEPAWorldModel

model = JEPAWorldModel(device='mps', num_samples=16, iterations=1)
if model.model is None:
    print('MODEL FAILED TO LOAD'); sys.exit(1)

tf = transforms.Compose([
    transforms.ToPILImage(), transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

def img_from(arr):
    return tf(np.clip(arr, 0, 255).astype(np.uint8)).unsqueeze(0).to(model.device)

obs = img_from(np.full((224, 224, 3), (96, 64, 48), dtype=np.uint8))
black = img_from(np.zeros((224, 224, 3), dtype=np.uint8))
white = img_from(np.full((224, 224, 3), 255, dtype=np.uint8))
noise = img_from(rng_arr := np.random.default_rng(1).integers(0, 256, (224, 224, 3)))

cases = []
def case(name, fn):
    cases.append((name, fn))

def expect_valid(name, fn):
    """action must be list of 4 finite floats in [-1,1]"""
    try:
        a = fn()
        ok = isinstance(a, list) and len(a) == 4 and all(
            isinstance(v, float) and np.isfinite(v) and -1.0 <= v <= 1.0 for v in a)
        return (name, ok, str([round(v,3) for v in a]) if ok else f'BAD: {a}')
    except Exception as e:
        return (name, False, f'EXC {str(e)[:60]}')

# 1-5 normal-ish variants
case('normal obs+goal', lambda: model.get_action(obs, black))
case('identity goal==obs', lambda: model.get_action(obs, obs))
case('white frame', lambda: model.get_action(white, black))
case('noise frame', lambda: model.get_action(noise, noise))
case('goal=white', lambda: model.get_action(obs, white))
# 6 proprio variants
case('proprio zeros', lambda: model.get_action(obs, black, proprio=[0,0,0,0]))
case('proprio realistic', lambda: model.get_action(obs, black, proprio=[0.15,0.0,0.15,1.0]))
case('proprio negative', lambda: model.get_action(obs, black, proprio=[-0.2,-0.3,-0.05,0.0]))
case('proprio long list (truncate)', lambda: model.get_action(obs, black, proprio=[0.1,0.1,0.1,1,99,'x']))
case('proprio short list', lambda: model.get_action(obs, black, proprio=[0.15]))
case('goal_proprio differs', lambda: model.get_action(obs, black,
        proprio=[0.15,0,0.15,1], goal_proprio=[0.20,0.10,0.06,0]))
# 9 alpha variations
for al in [0.0, 1.0, -0.5]:
    case(f'alpha={al}', lambda al=al: model.get_action(obs, black, alpha_proprio=al))
# 12-16 hostile tensors
case('float64 tensor', lambda: model.get_action(obs.double(), black.double()))
case('extreme values +-50', lambda: model.get_action((obs*50).clamp(-50,50), black))
case('batch=2', lambda: model.get_action(torch.cat([obs,obs]), torch.cat([black,black])))
case('cpu tensor input', lambda: model.get_action(obs.cpu(), black.cpu()))
case('NaN frame -> fallback zeros', lambda: model.get_action(
        torch.nan_to_num(torch.full_like(obs, float('nan'))), black))

results = [expect_valid(name, fn) for name, fn in cases]
passed = sum(1 for _, ok, _ in results if ok)
for name, ok, detail in results:
    print(f'{"PASS" if ok else "FAIL"} | {name} | {detail}')
print(f'TOTAL={len(results)} PASS={passed} FAIL={len(results)-passed}')
sys.exit(0 if passed == len(results) else 1)
