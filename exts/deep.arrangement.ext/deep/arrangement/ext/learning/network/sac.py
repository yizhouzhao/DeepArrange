import torch
import torch.nn as nn
from torchvision import models

from learning.network.resnet import ResNetFeatureExtractor

class Policy(nn.Module):
    def __init__(self, image_feature_dim = 512, object_feature_dim = 512) -> None:
        super().__init__()
        self.layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(image_feature_dim + object_feature_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 32),
            nn.ReLU(),
            nn.Linear(32, 2)
        )
    
    def forward(self, image_feature, object_feature):
        x = torch.cat([image_feature, object_feature], dim = 1)
        return self.layers(x)

class QFunction(nn.Module):
    def __init__(self, image_feature_dim = 512, object_feature_dim = 512, action_dim = 2) -> None:
        super().__init__()
        self.action_embedding = nn.Sequential(
            nn.Flatten(),
            nn.Linear(action_dim, 256),
        )
        self.feature_embedding = nn.Sequential(
            nn.Flatten(),
            nn.Linear(image_feature_dim + object_feature_dim, 256),
        )

        self.mlp = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )
    
    def forward(self, object_feature, image_feature, action):
        x1 = torch.cat([image_feature, object_feature], dim = 1)
        x1 = self.feature_embedding(x1)
        x2 = self.action_embedding(action)

        x = torch.cat([x1, x2], dim = 1)
        return self.mlp(x)
