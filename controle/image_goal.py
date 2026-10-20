"""Critère de terminaison image uniquement ; aucune commande de déplacement."""
import numpy as np
def perspective(points):
    p=np.atleast_2d(points.reshape(-1,8)).reshape(-1,4,2)
    left=np.linalg.norm(p[:,3]-p[:,0],axis=1);right=np.linalg.norm(p[:,2]-p[:,1],axis=1);width=np.linalg.norm(p[:,1]-p[:,0],axis=1)
    return (right-left)/np.maximum(right+left,.01)/np.maximum(width,.015)
def reached(z,goal,center_tolerance=.035):
    p=np.asarray(z).reshape(-1,4,2);g=np.asarray(goal).reshape(4,2)
    return (np.sqrt(np.mean((p-g)**2,axis=(1,2)))<.025)&(abs(p[:,:,0].mean(1)-g[:,0].mean())<center_tolerance)&(abs(perspective(p)-perspective(g))<.05)
