import numpy as np

class SwingFootTrajectory:
    def __init__(self, start_x, end_x, foot_y, duration=0.6, max_height=0.04):
        self.x0 = start_x
        self.x1 = end_x
        self.y  = foot_y
        self.T  = duration
        self.h  = max_height

    def at(self, t):
        t = np.clip(t, 0.0, self.T)
        phase = t / self.T
        x = self.x0 + (self.x1 - self.x0) * phase
        z = self.h * np.sin(np.pi * phase)
        return x, self.y, z

    def is_done(self, t):
        return t >= self.T


class FootstepPlanner:
    def __init__(self, step_length=0.06, foot_separation=0.08, n_steps=6):
        self.sl = step_length
        self.fs = foot_separation
        self.n  = n_steps

    def plan(self):
        footsteps = []
        x = 0.0
        for i in range(self.n):
            x += self.sl
            side = 'left' if i % 2 == 0 else 'right'
            y = self.fs/2 if side == 'left' else -self.fs/2
            footsteps.append((x, y, side))
        return footsteps
