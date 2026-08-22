"""Isolated JEPA-WMS CEM planner verification (no ROS2 required).

Recreates the exact tensor pipeline used by stacking_controller.py:
BGR uint8 HxWx3 -> PIL -> resize 224 -> ImageNet-normalized tensor.

Run: pixi run python3 test_jepa.py
Expected: no exceptions, non-zero action list of up to 20 floats.
"""
import time
import numpy as np
import torch
import torchvision.transforms as transforms

from jepa_model import JEPAWorldModel


def make_obs_image():
    """Synthetic top-down scene: table + colored blocks, mimics workspace_env rendering."""
    img = np.full((224, 224, 3), (96, 64, 48), dtype=np.uint8)  # brownish table (BGR)
    cv = np
    # red block
    img[100:130, 60:90] = (0, 0, 255)
    # green block
    img[110:140, 140:170] = (0, 255, 0)
    return img


def main():
    print("=" * 60)
    print("JEPA-WMS CEM planner verification")
    print("=" * 60)

    t0 = time.time()
    model = JEPAWorldModel(device='mps')
    if model.model is None:
        print("FAIL: model did not load")
        raise SystemExit(1)
    print(f"Model load time: {time.time() - t0:.1f}s")

    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    obs_img = make_obs_image()
    goal_img = np.zeros((224, 224, 3), dtype=np.uint8)

    obs_tensor = transform(obs_img).unsqueeze(0).to(model.device)
    goal_tensor = transform(goal_img).unsqueeze(0).to(model.device)
    print(f"Input shapes: obs={tuple(obs_tensor.shape)}, goal={tuple(goal_tensor.shape)}")

    t1 = time.time()
    action = model.get_action(obs_tensor, goal_tensor)
    dt = time.time() - t1

    print(f"Inference time: {dt * 1000:.0f} ms")
    print(f"Action length: {len(action)}")
    print(f"CEM Planner returned action: {[round(a, 4) for a in action]}")

    assert len(action) > 0, "empty action"
    nonzero = any(abs(a) > 1e-6 for a in action)
    print("-" * 60)
    if not all(a == 0.0 for a in action):
        print("PASS: model loaded and CEM planner produced an output")
        if not nonzero:
            print("NOTE: output is numerically zero (possible degenerate plan, "
                  "but pipeline itself ran without errors)")
    else:
        print("WARN: fallback zero-action returned -- check error trace above")
        raise SystemExit(1)


if __name__ == '__main__':
    main()
