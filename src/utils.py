#!/usr/bin/env python3

import numpy as np

def TL2B(q):
    return np.array([[2*(self.q[0]^2+self.q[1]^2)-1,2*(self.q[1]*self.q[2]-self.q[0]*self.q[3]),2*(self.q[1]*self.q[3]+self.q[0]*self.q[2])],
                     [2*(self.q[1]*self.q[2]+self.q[0]*self.q[3]),2*(self.q[0]^2+self.q[2]^2)-1,2*(self.q[2]*self.q[3]-self.q[0]*self.q[1])],
                     [2*(self.q[1]*self.q[3]-self.q[0]*self.q[2]),2*(self.q[2]*self.q[3]+self.q[0]*self.q[1]),2*(self.q[0]^2+self.q[3]^2)-1]])

def TB2L(q):
    return np.transpose(TL2B(q))

def normalize(v):
    vmag = norm(v,2)
    if vmag == 0:
        return v
    else:
        return v/vmag
    
def clip(val,min,max):
    np.minimum(max,np.maximum(min,val))
    