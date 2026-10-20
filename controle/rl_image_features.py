"""Interface acteur : uniquement coins normalisés et commandes effectivement envoyées."""
import numpy as np

def observation(corners,history):
    z=np.zeros(8) if corners is None else np.asarray(corners).reshape(8)
    past=np.asarray(history,dtype=float)
    if past.shape!=(10,2):raise ValueError('Dix commandes passées nécessaires')
    out=np.r_[z,past[::-1].reshape(20)].astype(np.float32)
    if not np.isfinite(out).all():raise ValueError('Observation non finie')
    return out

def velocity(action):
    a=np.asarray(action,dtype=float)
    if a.shape!=(2,) or not np.isfinite(a).all():raise ValueError('Action invalide')
    a=np.clip(a,-1,1)
    return np.array([a[0]*.07+.05,a[1]*.15])


def mirror_observation(features,center_u=.5):
    """Réflexion des coins identifiés, pas simple inversion de leur ordre."""
    x=np.asarray(features);single=x.ndim==1;rows=np.atleast_2d(x)
    if rows.shape[1]!=28:raise ValueError('28 entrées requises')
    out=rows.copy();q=rows[:,:8].reshape(-1,4,2)[:,[1,0,3,2],:].copy();q[:,:,0]=2*center_u-q[:,:,0]
    out[:,:8]=q.reshape(-1,8);out[:,9::2]*=-1
    return out[0] if single else out
