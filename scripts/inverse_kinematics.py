import numpy as np

L = 0.20  # both links equal length (symmetric leg)

def leg_ik(h):
    h = np.clip(h, 0.01, 2*L - 0.001)
    q1 = np.arccos(h / (2*L))
    q2 = -2 * q1
    ankle = q1
    return q1, q2, ankle

class LegIKSolver:
    def __init__(self, side='left'):
        self.side = side
        self.y_sign = 1.0 if side == 'left' else -1.0

    def solve(self, com_z, foot_x=0.0, foot_z_world=0.0, hip_roll=0.0):
        h = np.clip(com_z - foot_z_world, 0.01, 2*L - 0.001)
        q1, q2, ankle = leg_ik(h)
        if abs(foot_x) > 1e-4:
            lean = np.arctan2(foot_x, h)
            q1 = np.clip(q1 + lean, -1.57, 0.52)
        return {
            'hip_roll':  float(np.clip(hip_roll, -0.4, 0.4)),
            'hip_pitch': float(q1),
            'knee':      float(q2),
            'ankle':     float(ankle),
        }
