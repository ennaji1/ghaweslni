"""Politique distillée : réseaux uniquement pour v/omega ; arrêt image externe."""
import os
from pathlib import Path
import numpy as np
import torch
from torch import nn
from gymnasium import spaces
from stable_baselines3.common.policies import ActorCriticPolicy
from ppo_image_features import expand,CornerHistoryExtractor
from rl_image_features import mirror_observation
from image_goal import reached
SCALE=np.array([.07,.15],dtype=np.float32)
class ResidualNet(nn.Module):
    def __init__(self):
        super().__init__();self.register_buffer('mean',torch.zeros(32));self.register_buffer('std',torch.ones(32))
        self.layers=nn.Sequential(nn.Linear(32,128),nn.Tanh(),nn.Linear(128,64),nn.Tanh(),nn.Linear(64,2))
        nn.init.zeros_(self.layers[-1].weight);nn.init.zeros_(self.layers[-1].bias)
    def fit(self,x):
        with torch.no_grad():
            z=expand(x);self.mean.copy_(z.mean(0));self.std.copy_(z.std(0).clamp(min=.001))
    def forward(self,x):return self.layers(((expand(x)-self.mean)/self.std).clamp(-10,10))
def create_base():
    return ActorCriticPolicy(spaces.Box(-10,10,(28,),dtype=np.float32),spaces.Box(-1,1,(2,),dtype=np.float32),lambda _:0.,net_arch=dict(pi=[64,64],vf=[64,64]),features_extractor_class=CornerHistoryExtractor)
def base_commands(base,x,center):
    x=np.atleast_2d(x).astype(np.float32)
    a,_=base.predict(x,deterministic=True);b,_=base.predict(mirror_observation(x,center),deterministic=True);b[:,1]*=-1;a=(a+b)/2
    return a*SCALE+np.array([.05,0],dtype=np.float32)
def residual_commands(net,x,center):
    x=torch.as_tensor(x,dtype=torch.float32)
    mirrored=x.reshape(-1,28).clone();q=x[:,:8].reshape(-1,4,2)[:,[1,0,3,2],:].clone();q[:,:,0]=2*center-q[:,:,0];mirrored[:,:8]=q.reshape(-1,8);mirrored[:,9::2]*=-1
    both=net(torch.cat((x,mirrored)));a,b=both.chunk(2);return (a+b*torch.tensor([1.,-1.]))/2
class Policy:
    indices=np.arange(28)
    @classmethod
    def load(cls,path):
        torch.set_num_threads(1);data=torch.load(str(path),map_location='cpu',weights_only=True);p=cls();p.base=create_base();p.base.load_state_dict(data['base']);p.base.eval();p.residual=ResidualNet();p.residual.load_state_dict(data['residual']);p.residual.eval();p.goal=data['goal'].numpy();p.center=float(data['center']);p.finished=False;p.stop_center=float(os.environ.get("PPO_STOP_CENTER_TOL",".035"));return p
    def raw_prediction(self,features):
        x=np.atleast_2d(features).astype(np.float32)
        with torch.no_grad():delta=residual_commands(self.residual,x,self.center).numpy()*SCALE
        return np.clip(base_commands(self.base,x,self.center)+delta,[-.02,-.15],[.12,.15])
    def predict(self,features):
        x=np.atleast_2d(features)
        if len(x)!=1:raise ValueError('Un seul instant à la fois')
        if reached(x[0,:8],self.goal,self.stop_center)[0]:self.finished=True
        return np.zeros((1,2)) if self.finished else self.raw_prediction(x)
