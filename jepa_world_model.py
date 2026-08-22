import torch
import torch.nn as nn

class JEPAEncoder(nn.Module):
    def __init__(self, latent_dim=128):
        super().__init__()
        # Simple CNN encoder for dummy 64x64 images
        self.net = nn.Sequential(
            nn.Conv2d(3, 32, 4, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, 4, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 128, 4, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 256, 4, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(256 * 4 * 4, latent_dim)
        )

    def forward(self, x):
        return self.net(x)

class JEPAPredictor(nn.Module):
    def __init__(self, latent_dim=128, action_dim=4):
        super().__init__()
        # Predicts next state given current state and action
        self.net = nn.Sequential(
            nn.Linear(latent_dim + action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, latent_dim)
        )

    def forward(self, s, a):
        x = torch.cat([s, a], dim=-1)
        return self.net(x)

class JEPAPolicy(nn.Module):
    def __init__(self, latent_dim=128, action_dim=4):
        super().__init__()
        # Generates action given state
        self.net = nn.Sequential(
            nn.Linear(latent_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh() # Actions bounded between -1 and 1
        )

    def forward(self, s):
        return self.net(s)

class JEPAWorldModel(nn.Module):
    def __init__(self, latent_dim=128, action_dim=4):
        super().__init__()
        self.encoder = JEPAEncoder(latent_dim)
        self.predictor = JEPAPredictor(latent_dim, action_dim)
        self.policy = JEPAPolicy(latent_dim, action_dim)

    def get_action(self, obs):
        s = self.encoder(obs)
        a = self.policy(s)
        return a
