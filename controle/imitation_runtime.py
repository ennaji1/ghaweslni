"""Etat d'inference : coins detectes + commandes passees, aucune pose externe."""
from collections import deque
import numpy as np

class Runtime:
    def __init__(self,policy):
        self.policy=policy;self.history=deque(maxlen=300);self.corners=None;self.image_stamp=None
        self.last_used=None;self.last_output=(0.,0.);self.features=None;self.reason='ATTENTE CAMERA'
    def observe(self,corners,stamp,width,height):
        self.image_stamp=int(stamp);self.corners=None
        if corners is None:self.reason='PAUSE : MARQUEUR NON DETECTE';return
        points=np.asarray(corners,dtype=float)
        if points.shape!=(4,2) or not np.isfinite(points).all() or np.any(points<0) or np.any(points[:,0]>=width) or np.any(points[:,1]>=height):
            self.reason='PAUSE : COINS INVALIDES';return
        self.corners=(points/np.array([width,height])).reshape(8)
    def record_sent(self,stamp,v,w):self.history.append((int(stamp),float(v),float(w)))
    def command(self,stamp):
        self.features=None
        if self.corners is None or self.image_stamp is None:return (0.,0.)
        age=(stamp-self.image_stamp)*1e-9
        if not -.05<=age<=.3:self.reason='PAUSE : IMAGE ANCIENNE';return (0.,0.)
        past=[item for item in self.history if item[0]<self.image_stamp]
        if len(past)<10:self.reason='INITIALISATION : HISTORIQUE ZERO';return (0.,0.)
        if self.image_stamp==self.last_used:
            self.reason='MODELE : MAINTIEN COMMANDE';return self.last_output
        features=np.r_[self.corners,np.array([item[1:] for item in past[-10:][::-1]]).reshape(20)]
        output=np.asarray(self.policy.predict(features))[0]
        if output.shape!=(2,) or not np.isfinite(output).all():self.reason='ARRET : SORTIE MODELE INVALIDE';return (0.,0.)
        self.features=features;self.last_used=self.image_stamp;self.last_output=tuple(map(float,output))
        self.reason='MODELE : COMMANDE PREDITE'
        return self.last_output
