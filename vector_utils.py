import numpy as np
from numpy import pi
from numpy import sqrt

def dot_vec(pa,pb):
    return pa[0]*pb[0]+pa[1]*pb[1]

def ang_vec(p):
    return np.arctan2(p[1],p[0])

def ang_vec_diff(pa,pb):
    a = ang_vec(pa)
    b = ang_vec(pb)
    return ang_diff(a,b)

def targ_diff(pa,pb,targ):
    a = ang_vec(pa)
    b = ang_vec(pb)
    diff =  (b-a)%(2*pi) if targ % 2 else (a-b)%(2*pi)
    if diff > pi:
        diff -= pi
    if diff < -pi:
        diff += pi
    return diff

def ang_diff(a,b):
    return min(2*pi - (b-a)%(2*pi), (b-a)%(2*pi))

def vec_norm(p1,p2):
    vec = p1 - p2
    return vec, sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def ortogonal_vec(v):
    return np.array([-v[1],v[0]])
