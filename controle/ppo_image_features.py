"""Transformation interne des 28 entrées ; statistiques ajustées au train seulement."""
import numpy as np
import torch
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

def expand(x):
    p=x[:,:8].reshape(-1,4,2);center=p.mean(dim=1)
    left=torch.linalg.vector_norm(p[:,3]-p[:,0],dim=1);right=torch.linalg.vector_norm(p[:,2]-p[:,1],dim=1)
    height=(left+right)/2;width=torch.linalg.vector_norm(p[:,1]-p[:,0],dim=1)
    skew=(right-left)/(right+left).clamp(min=.01)/width.clamp(min=.015)
    return torch.cat((x,center,torch.log(height.clamp(min=.02))[:,None],skew[:,None]),dim=1)

class CornerHistoryExtractor(BaseFeaturesExtractor):
    def __init__(self,observation_space):
        super().__init__(observation_space,features_dim=32)
        self.register_buffer('mean',torch.zeros(32));self.register_buffer('std',torch.ones(32))
    def fit(self,train):
        with torch.no_grad():
            z=expand(train);self.mean.copy_(z.mean(dim=0));self.std.copy_(z.std(dim=0).clamp(min=.001))
    def forward(self,observations):return ((expand(observations)-self.mean)/self.std).clamp(-10,10)
