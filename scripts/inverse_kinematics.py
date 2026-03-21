import numpy as np

L1 = 0.20
L2 = 0.20
ANKLE_HEIGHT = 0.01

def leg_ik(foot_x, foot_z):
    r = np.sqrt(foot_x**2 + foot_z**2)
    r = np.clip(r, abs(L1-L2)+1e-4, L1+L2-1e-4)
    cos_knee = (r**2 - L1**2 - L2**2) / (2*L1*L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee = np.arccos(cos_knee)
    alpha = np.arctan2(foot_x, -foot_z)
    beta  = np.arccos(np.clip((r**2+L1**2-L2**2)/(2*r*L1), -1.0, 1.0))
    hip_pitch = alpha - beta
    hip_pitch = np.clip(hip_pitch, -1.57, 0.52)
    knee      = np.clip(knee, 0.0, 2.618)
    return hip_pitch, knee

def ankle_compensation(hip_pitch, knee):
    ankle = -(hip_pitch + knee*0.3)
    return np.clip(ankle, -0.5, 0.5)

class LegIKSolver:
    HIP_Z_OFFSET = -0.09

    def __init__(self, side='left'):
        self.side   = side
        self.y_sign = 1.0 if side == 'left' else -1.0

    def solve(self, com_z, foot_x, foot_z_world, hip_roll=0.0):
        # hip_pitch joint is at com_z height above ground
        rel_x = foot_x
        rel_z = foot_z_world - com_z
        hip_pitch, knee = leg_ik(rel_x, rel_z)
        ankle = ankle_compensation(hip_pitch, knee)
        return {
            'hip_roll':  float(np.clip(hip_roll, -0.4, 0.4)),
            'hip_pitch': float(hip_pitch),
            'knee':      float(knee),
            'ankle':     float(ankle),
        }
