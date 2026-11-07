"""Portable experimental reproduction of the F103 raw-position PID."""
import math
import numpy as np


class FirmwarePID:
    def __init__(self):
        self.previous = None
        self.integral = np.float32(0)

    def step(self, goal, measured):
        f = np.float32
        if not (0 <= goal <= 4095 and 0 <= measured <= 4095):
            raise ValueError('F103 position outside 0..4095')
        if self.previous is None:
            self.previous = f(measured)
        dt=f(.02)
        velocity=f(f(f(measured)-self.previous)/dt)
        self.previous=f(measured)
        error=f(f(goal)-f(measured))
        candidate=f(np.clip(f(self.integral+f(f(f(3)*error)*dt)),-100,100))
        without_i=f(f(f(7)*error)-f(f(.13)*velocity))
        unsaturated=f(without_i+candidate)
        if not ((unsaturated>1000 and error>0) or (unsaturated<-1000 and error<0)):
            self.integral=candidate
        requested=f(np.clip(f(without_i+self.integral),-1000,1000))
        self.p_term=float(f(f(7)*error))
        self.i_term=float(self.integral)
        self.d_term=float(f(-f(f(.13)*velocity)))
        self.error=float(error)
        self.requested=float(requested)
        output=float(f(-requested))
        return int(math.copysign(math.floor(abs(output)+.5),output))
