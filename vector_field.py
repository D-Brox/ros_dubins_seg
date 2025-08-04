import numpy as np
from numpy import cos,sin,pi

from vector_utils import ang_vec, vec_norm, ortogonal_vec

class CircleVectorField():

    def __init__(self, r, cx, cy, dir = 1.0, vr = 1.0):
        self.c_r = [np.array([cx,cy]),r]
        self.r = lambda s,r: np.array([r*np.cos(s)+cx, r*sin(s)+cy])
        self.direction = dir/abs(dir)
        self.vr = vr

    def compute_field(self,pose):
        p = np.array([pose[0], pose[1]])
        eta = self.vr

        c,r = self.c_r
        p_c,p_c_norm = vec_norm(p,c)
        s_star = ang_vec(p_c)
        r_star = self.r(s_star,r)

        D, dnorm = vec_norm(p,r_star)
        if dnorm != 0:
            N, norm = vec_norm(D,0.5*D/dnorm)
            if norm == 0:
                norm = 1e-3
            T = self.direction*ortogonal_vec(p_c)/p_c_norm
            G = - norm/np.sqrt(1 + norm**2)
            H = 1/np.sqrt(1+norm**2)
        else:
            N = D
            T = self.direction*ortogonal_vec(p_c)/p_c_norm
            G = 0
            H = 1

        F =  eta*(G*N+H*T)
        delta = p_c_norm/r
        return F,D,T,G,H,delta

    def redefine(self, r, cx, cy, dir = None, kG = None, vr = None):
        self.c_r = [np.array([cx,cy]),r]
        self.r = lambda s,r: np.array([r*cos(s)+cx, r*sin(s)+cy])
        if dir is not None:
            self.direction = dir/abs(dir)
        if vr is not None:
            self.vr = vr
            
    def compute_theta(self,x,y):
        [cx,cy] = self.c_r[0]
        dir = self.direction
        ang = np.arctan2(y-cy,x-cx)
        return ang - dir*np.pi/2
    
    def compute_next(self,pose,v,dt):
        p = pose[:2]
        c,r = self.c_r
        p_c,p_c_norm = vec_norm(p,c)
        rh = (p_c_norm + r)/2
        s_star = ang_vec(p_c) - self.direction * (v/r)*dt
        r_star = self.r(s_star,rh)
        r_ang = ang_vec(r_star) - self.direction * pi/2
        return r_star[0],r_star[1],r_ang
        