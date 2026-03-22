import numpy as np
from scipy.linalg import solve_discrete_are

class LIPMPreviewController:
    def __init__(self, dt=0.02, com_height=0.38, preview_horizon=20):
        self.dt = dt
        self.g  = 9.81
        self.h  = com_height
        self.N  = preview_horizon
        self.omega = np.sqrt(self.g / self.h)

        T = dt
        w = self.omega
        self.A = np.array([
            [np.cosh(T*w),   np.sinh(T*w)/w, 1-np.cosh(T*w)],
            [w*np.sinh(T*w), np.cosh(T*w),   -w*np.sinh(T*w)],
            [0,              0,              1             ]
        ])
        self.B = np.array([
            [T - np.sinh(T*w)/w],
            [1 - np.cosh(T*w)  ],
            [T                 ]
        ])
        self.C = np.array([[1, 0, -self.h/self.g]])

        Qe = 1.0; R = 1e-3
        A_aug = np.zeros((4,4))
        A_aug[0,0] = 1.0
        A_aug[0,1:4] = (self.C @ self.A).flatten()
        A_aug[1:4,1:4] = self.A
        B_aug = np.zeros((4,1))
        B_aug[0,0] = float(np.squeeze(self.C @ self.B))
        B_aug[1:4,0] = self.B.flatten()
        Q_aug = np.zeros((4,4)); Q_aug[0,0] = Qe
        P = solve_discrete_are(A_aug, B_aug, Q_aug, np.array([[R]]))
        K = np.linalg.inv(R + B_aug.T @ P @ B_aug) @ B_aug.T @ P @ A_aug
        self.Ke = float(K[0,0])
        self.Kx = K[0,1:4].flatten()
        Acl = A_aug - B_aug @ K
        self.Gp = np.zeros(self.N)
        tmp = np.linalg.inv(R + B_aug.T @ P @ B_aug) @ B_aug.T
        vec = np.zeros((4,1))
        vec[0,0] = float(np.squeeze(self.C @ self.B))
        vec[1:4,0] = self.B.flatten()
        power = np.eye(4)
        for i in range(self.N):
            self.Gp[i] = float(np.squeeze(tmp @ power @ P @ vec))
            power = power @ Acl
        self.x = np.zeros(3); self.e = 0.0

    def reset(self, com_pos=0.0):
        self.x = np.array([com_pos, 0.0, 0.0]); self.e = 0.0

    def step(self, zmp_ref_preview):
        zmp_now = float(np.squeeze(self.C @ self.x))
        self.e += zmp_now - float(zmp_ref_preview[0])
        preview_sum = sum(self.Gp[i]*zmp_ref_preview[i] for i in range(self.N))
        u = -self.Ke*self.e - float(np.squeeze(self.Kx @ self.x)) - preview_sum
        self.x = self.A @ self.x + self.B.flatten() * float(u)
        return float(self.x[0])

    @property
    def com_pos(self): return float(self.x[0])
    @property
    def com_vel(self): return float(self.x[1])
    @property
    def zmp(self): return float(np.squeeze(self.C @ self.x))
    @property
    def dcm(self): return self.com_pos + self.com_vel / self.omega

class ZMPReferenceGenerator:
    def __init__(self, dt=0.02, step_duration=0.6, ds_duration=0.1,
                 step_length=0.12, foot_separation=0.08):
        self.dt=dt; self.Tss=step_duration; self.Tds=ds_duration
        self.sl=step_length; self.fs=foot_separation
        self.Nss=int(self.Tss/dt); self.Nds=int(self.Tds/dt)

    def generate(self, n_steps=6):
        zmp_x, zmp_y = [], []
        foot_x, foot_y = 0.0, self.fs/2
        for step_idx in range(n_steps):
            side = 1 if step_idx % 2 == 0 else -1
            foot_x += self.sl; foot_y = side * self.fs/2
            prev_x = zmp_x[-1] if zmp_x else 0.0
            prev_y = zmp_y[-1] if zmp_y else 0.0
            for k in range(self.Nds):
                t = k / max(self.Nds-1, 1)
                zmp_x.append(prev_x + t*(foot_x - prev_x))
                zmp_y.append(prev_y + t*(foot_y - prev_y))
            for _ in range(self.Nss):
                zmp_x.append(foot_x); zmp_y.append(foot_y)

        return np.array(zmp_x), np.array(zmp_y)
