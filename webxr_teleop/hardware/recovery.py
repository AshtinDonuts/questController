"""Restart readiness uses encoder stability, not agreement with a failed hold goal."""
from collections import deque
import math
from .core import FEEDBACK_TTL


class RestartWindow:
    def __init__(self):
        self.samples=deque()
        self.ready=False

    def update(self,q,t):
        if self.samples and t==self.samples[-1][0]:return self.ready
        if self.samples and (t<self.samples[-1][0] or t-self.samples[-1][0]>FEEDBACK_TTL):self.samples.clear()
        self.samples.append((t,q[:]))
        while len(self.samples)>1 and self.samples[1][0]<=t-.3:self.samples.popleft()
        tolerance=[4*math.pi/4096+1e-9]*6+[.02]
        self.ready=(t-self.samples[0][0]>=.3-1e-9 and all(
            max(v[j] for _,v in self.samples)-min(v[j] for _,v in self.samples)<=tolerance[j] for j in range(7)))
        return self.ready
