import numpy as np

L = 0.20  # both links equal length (symmetric leg)
G = 9.81  # gravity

def leg_ik(h):
    """
    Inverse kinematics for symmetric leg from Stéphane Caron's blog.
    Given crouching height h, returns (hip, knee) angles.
    L1 = L2 = L = 0.20m
    
    q1 = arccos(h / 2L)   hip angle
    q2 = -2 * q1          knee angle (symmetric)
    ankle = q1             keeps foot flat
    """
    # Clamp h to valid range
    h = np.clip(h, 0.01, 2*L - 0.001)
    q1 = np.arccos(h / (2*L))      # hip
    q2 = -2 * q1                    # knee
    ankle = q1                      # flat foot compensation
    return q1, q2, ankle

class LegIKSolver:
    def __init__(self, side='left'):
        self.side = side
        self.y_sign = 1.0 if side == 'left' else -1.0

    def solve(self, com_z, foot_x=0.0, foot_z_world=0.0, hip_roll=0.0):
        """
        Solve IK for leg.
        com_z: COM height above ground
        foot_x: forward offset of foot from COM
        foot_z_world: foot height (0 = ground)
        hip_roll: lateral tilt
        """
        # Effective height = COM height minus foot height
        h = com_z - foot_z_world

        # Get joint angles from symmetric IK
        q1, q2, ankle = leg_ik(h)

        # Add forward lean correction for foot_x offset
        if abs(foot_x) > 1e-4:
            lean = np.arctan2(foot_x, h)
            q1 = q1 + lean
            q1 = np.clip(q1, -1.57, 0.52)

        return {
            'hip_roll':  float(np.clip(hip_roll, -0.4, 0.4)),
            'hip_pitch': float(q1),
            'knee':      float(np.clip(-2*np.arccos(np.clip(h/(2*L), 0.01, 0.999)), -2.618, 0.0)),
            'ankle':     float(np.clip(ankle, -1.2, 1.2)),
        }

if __name__ == '__main__':
    ik = LegIKSolver('left')
    for h in [0.38, 0.35, 0.32, 0.30]:
        j = ik.solve(h, 0.0, 0.0)
        total = j['hip_pitch'] + j['knee'] + j['ankle']
        print(f"h={h}: hip={np.degrees(j['hip_pitch']):.1f}° "
              f"knee={np.degrees(j['knee']):.1f}° "
              f"ankle={np.degrees(j['ankle']):.1f}° "
              f"flat={total:.4f}")
