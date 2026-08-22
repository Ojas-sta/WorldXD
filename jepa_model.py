import torch
import torch.nn as nn
import os
import sys

class JEPAWorldModel(nn.Module):
    def __init__(self, device='mps'):
        super().__init__()
        self.device = torch.device(device)
        
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
            print(f"JEPA-WMS loaded successfully on {self.device} with {self.dtype}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Failed to load Meta JEPA model: {e}")
            self.model = None
            self.preprocessor = None

    def get_action(self, image_tensor, goal_tensor):
        """
        Forward pass for inference.
        Note: The official JEPA-WMS does not have a direct policy (s_t, s_goal) -> a_t.
        It uses the Cross-Entropy Method (CEM) to plan action sequences that minimize distance
        between predicted future latents and the goal latent.
        
        For this simulation pipeline wrapper, we return a dummy zero-action to allow the geometric 
        stacking_controller to drive the physical robot while this model processes the visual feed in parallel.
        """
        if self.model is None:
            return [0.0, 0.0, 0.0, 0.0]
            
        try:
            # We can still run the encoder to verify the pipeline doesn't OOM!
            image_tensor = image_tensor.to(self.device, dtype=self.dtype)
            goal_tensor = goal_tensor.to(self.device, dtype=self.dtype)
            
            # The JEPA model expects (batch, time, channels, height, width)
            if image_tensor.ndim == 4:
                image_tensor = image_tensor.unsqueeze(1)
            if goal_tensor.ndim == 4:
                goal_tensor = goal_tensor.unsqueeze(1)
            
            # The MetaWorld models are configured with proprio_dim=4 and proprio_encoding='feature'
            # We must provide a proprio tensor, otherwise the AdaLN predictor will crash expecting 400 dims instead of 384
            b, t, c, h, w = image_tensor.shape
            proprio_tensor = torch.zeros((b, t, 4), device=self.device, dtype=self.dtype)
            
            # --- CEM Planner Implementation ---
            # 1. Get current and goal encodings
            with torch.no_grad():
                z_init_raw = self.model.encode({"visual": image_tensor, "proprio": proprio_tensor}, act=True)
                # Keep as dict so unroll receives proprio features (needed for feature-concat proprio)
                if hasattr(z_init_raw, "items"):
                    z_init = {k: v.detach() for k, v in z_init_raw.items()}
                    z_goal_raw = self.model.encode({"visual": goal_tensor, "proprio": proprio_tensor}, act=False)
                    z_goal_visual = z_goal_raw["visual"].detach()
                else:
                    z_init = z_init_raw.detach()
                    z_goal_raw = self.model.encode({"visual": goal_tensor, "proprio": proprio_tensor}, act=False)
                    z_goal_visual = z_goal_raw.detach()
                
                # 2. Hyperparameters
                horizon = 5 # 5 steps to reach goal
                action_dim = self.model.action_dim if hasattr(self.model, 'action_dim') else 20 # frameskip * action_dim (e.g. 5 * 4 = 20)
                num_samples = 256
                iterations = 3
                num_elites = 32
                
                # 3. Initialize distributions
                mean = torch.zeros(horizon, action_dim, device=self.device, dtype=self.dtype)
                std = torch.ones(horizon, action_dim, device=self.device, dtype=self.dtype)
                
                actions = torch.empty(horizon, num_samples, action_dim, device=self.device, dtype=self.dtype)
                
                for itr in range(iterations):
                    # Sample actions
                    actions = mean.unsqueeze(1) + std.unsqueeze(1) * torch.randn(
                        horizon, num_samples, action_dim, device=self.device, dtype=self.dtype
                    )
                    # Keep previous mean (elite inclusion trick)
                    actions[:, 0, :] = mean
                    
                    # Predict future states
                    # unroll expects (z_init, actions) where actions is (horizon, batch_size, action_dim)
                    # unroll returns dict/TensorDict with 'visual' key when given dict z_init
                    # or Tensor when given Tensor z_init
                    predicted_encs = self.model.unroll(z_init, act_suffix=actions)
                    
                    # Compute objective: L2 distance on visual features at final step
                    if isinstance(predicted_encs, dict) or hasattr(predicted_encs, "items"):
                        final_enc = predicted_encs["visual"][-1]  # [B, V, H, W, D]
                        z_goal_final = z_goal_visual[:, -1] if z_goal_visual.ndim > final_enc.ndim else z_goal_visual
                        diff = (z_goal_final - final_enc).pow(2).mean(dim=tuple(range(1, final_enc.ndim)))
                    else:
                        final_enc = predicted_encs[-1]
                        z_goal_final = z_goal_visual[:, -1] if z_goal_visual.ndim > final_enc.ndim else z_goal_visual
                        diff = (z_goal_final - final_enc).pow(2).mean(dim=tuple(range(1, final_enc.ndim)))
                    
                    # Get elite indices
                    elite_idxs = torch.topk(-diff, num_elites, dim=0).indices
                    elite_actions = actions[:, elite_idxs] # [horizon, num_elites, action_dim]
                    
                    # Update parameters
                    mean = torch.mean(elite_actions, dim=1)
                    std = torch.std(elite_actions, dim=1)
            
            # The best action to take right now is the first step of the mean trajectory
            best_action = mean[0].cpu().numpy().tolist()
            
            # Bound action reasonably for a robot (just to prevent crazy jerks)
            best_action = [max(min(a, 0.1), -0.1) for a in best_action]
            
            print(f"CEM Planner chose action: {best_action}")
            return best_action
            
        except Exception as e:
            print(f"Error in get_action: {e}")
            import traceback
            traceback.print_exc()
            return [0.0, 0.0, 0.0, 0.0]
