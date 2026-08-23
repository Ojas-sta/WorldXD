import torch
import torch.nn as nn
import os
import sys

class JEPAWorldModel(nn.Module):
    def __init__(self, device='mps', num_samples=256, iterations=3):
        super().__init__()
        self.device = torch.device(device)
        self.num_samples = num_samples
        self.iterations = iterations

        # We load it using float16 to save memory on 16GB Mac
        self.dtype = torch.float16

        print("Loading official facebook/jepa-wms from local clone...")

        try:
            repo_dir = os.path.join(os.path.dirname(__file__), 'jepa-wms')

            # The official repo loads from checkpoint and instantiates the EncPredWM architecture.
            # We use torch.hub for convenience since the repo implements hubconf.py
            self.model, self.preprocessor = torch.hub.load(
                repo_dir,
                'jepa_wm_metaworld',
                source='local',
                pretrained=True,
                device='cpu'  # Load to CPU first to avoid MPS initialization issues during graph building
            )

            # Move to target device and cast to half precision
            self.model = self.model.to(self.device, dtype=self.dtype)
            self.model.eval()

            # P4: normalization stats live on the preprocessor; move copies to our
            # device so denormalization happens in the same space as planning.
            stats = self.preprocessor
            self.action_mean = stats.action_mean.float().to(self.device)
            self.action_std = stats.action_std.float().to(self.device)
            print(f"JEPA-WMS loaded successfully on {self.device} with {self.dtype}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Failed to load Meta JEPA model: {e}")
            self.model = None
            self.preprocessor = None

    def get_action(self, image_tensor, goal_tensor, proprio=None, goal_proprio=None,
                   alpha_proprio=0.1):
        """
        Plan one raw action with the JEPA world model via CEM.

        Reference semantics (jepa-wms planner.py / objectives.py / plan_evaluator.py):
          - actions planned in NORMALIZED space ((a-mean)/std), no clamping in CEM
          - cost = L2(final visual latent) + alpha * L2(final proprio latent),
            final timestep only (sum_all_diffs=False)
          - executed action = denormalized first chunk of the frameskip-flattened
            20-dim vector ('t (f d) -> (t f) d' with d=4)

        Args:
            image_tensor: [1,3,H,W] or [1,1,3,H,W] normalized observation
            goal_tensor:  same layout as image_tensor
            proprio:      list/tensor of RAW proprio [x, y, z, gripper_open(0/1)];
                          encode() normalizes internally. Zeros are out-of-distribution.
            goal_proprio: optional raw proprio at the goal; defaults to `proprio`.
            alpha_proprio: weight of the proprio term (reference config uses 0.1)

        Returns:
            [dx, dy, dz, gripper] denormalized raw action, bounded to [-1, 1].
            Falls back to zeros on any error (never crashes the controller).
        """
        if self.model is None:
            return [0.0, 0.0, 0.0, 0.0]

        try:
            # Defensive coercion: fp64 tensors crash MPS ("cannot convert MPS
            # tensor to float64") — route everything through fp32 first.
            image_tensor = image_tensor.detach().to(
                device=self.device, dtype=torch.float32).to(self.dtype)
            goal_tensor = goal_tensor.detach().to(
                device=self.device, dtype=torch.float32).to(self.dtype)

            if image_tensor.ndim == 4:
                image_tensor = image_tensor.unsqueeze(1)
            if goal_tensor.ndim == 4:
                goal_tensor = goal_tensor.unsqueeze(1)

            b, t, c, h, w = image_tensor.shape

            # P4: real proprio instead of zeros. encode() normalizes with the
            # dataset stats internally (vit_enc_preds.py normalize_proprios).
            if proprio is None:
                proprio = [0.0] * 4
            prop = torch.tensor([float(v) for v in list(proprio)[:4]],
                                device=self.device, dtype=self.dtype)
            proprio_tensor = prop.view(1, 1, 4).expand(b, t, 4)

            if goal_proprio is None:
                goal_proprio = proprio
            gp = torch.tensor([float(v) for v in list(goal_proprio)[:4]],
                              device=self.device, dtype=self.dtype)
            goal_proprio_tensor = gp.view(1, 1, 4).expand(b, t, 4)

            # --- CEM Planner ---
            with torch.no_grad():
                z_init_raw = self.model.encode(
                    {"visual": image_tensor, "proprio": proprio_tensor}, act=True)
                z_goal_raw = self.model.encode(
                    {"visual": goal_tensor, "proprio": goal_proprio_tensor}, act=False)

                # keep plain dicts of detached tensors
                if hasattr(z_init_raw, "items"):
                    z_init = {k: v.detach() for k, v in z_init_raw.items()}
                    z_goal = {k: v.detach() for k, v in z_goal_raw.items()}
                    z_goal_visual = z_goal["visual"]
                    z_goal_proprio = z_goal.get("proprio")
                else:
                    z_init = z_init_raw.detach()
                    z_goal_visual = z_goal_raw.detach()
                    z_goal_proprio = None

                horizon = 5
                action_dim = self.model.action_dim if hasattr(self.model, 'action_dim') else 20
                num_samples = self.num_samples
                iterations = self.iterations
                num_elites = max(8, num_samples // 8)

                mean = torch.zeros(horizon, action_dim, device=self.device, dtype=self.dtype)
                std = torch.ones(horizon, action_dim, device=self.device, dtype=self.dtype)

                for itr in range(iterations):
                    actions = mean.unsqueeze(1) + std.unsqueeze(1) * torch.randn(
                        horizon, num_samples, action_dim, device=self.device, dtype=self.dtype)
                    actions[:, 0, :] = mean   # elite inclusion trick

                    predicted_encs = self.model.unroll(z_init, act_suffix=actions)

                    # Objective: L2 on FINAL timestep latents, visual + alpha*proprio
                    has_items = hasattr(predicted_encs, "items")
                    final_vis = predicted_encs["visual"][-1] if has_items else predicted_encs[-1]
                    gv = z_goal_visual[:, -1] if z_goal_visual.ndim > final_vis.ndim else z_goal_visual
                    diff = (gv - final_vis).pow(2).mean(dim=tuple(range(1, final_vis.ndim)))

                    if has_items and z_goal_proprio is not None and "proprio" in predicted_encs:
                        final_prop = predicted_encs["proprio"][-1]
                        gprop = z_goal_proprio[:, -1]
                        diff_p = (gprop - final_prop).pow(2).mean(dim=tuple(range(1, final_prop.ndim)))
                        diff = diff + alpha_proprio * diff_p

                    elite_idxs = torch.topk(-diff, num_elites, dim=0).indices
                    elite_actions = actions[:, elite_idxs]
                    mean = torch.mean(elite_actions, dim=1)
                    std = torch.std(elite_actions, dim=1)

            # Denormalize: planning space -> metaworld raw action space
            seq = mean[0].float().view(-1, 4)          # [t (f d)] -> [(t f) d]
            raw_vec = (seq[0] * self.action_std + self.action_mean)
            raw = raw_vec.cpu().numpy().tolist()

            # Robot-sane bounds AFTER denormalization (metaworld raw range ~[-1.5, 1.5])
            raw = [max(min(a, 1.0), -1.0) for a in raw]

            print(f"CEM Planner chose action: {[round(a, 4) for a in raw]}")
            return raw

        except Exception as e:
            print(f"Error in get_action: {e}")
            import traceback
            traceback.print_exc()
            return [0.0, 0.0, 0.0, 0.0]
