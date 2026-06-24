import numpy as np


class KalmanFilter:

    def __init__(self, dt):

        self.dt = dt

        # State:
        # x = [x, y, psi]

        self.x = np.zeros((3, 1))

        self.P = np.eye(3)

        self.A = np.eye(3)

        self.B = np.zeros((3, 2))

        self.H = np.eye(3)

        self.Q = np.diag([
            0.01,
            0.01,
            0.001
        ])

        self.R = np.diag([
            0.2**2,
            0.2**2,
            0.03**2
        ])

    def predict(self, v, delta, L):

        psi = self.x[2, 0]

        self.x[0, 0] += (
            v*np.cos(psi)*self.dt
        )

        self.x[1, 0] += (
            v*np.sin(psi)*self.dt
        )

        self.x[2, 0] += (
            v/L*np.tan(delta)*self.dt
        )

        self.P = (
            self.A @ self.P @ self.A.T
            + self.Q
        )

    def update(
        self,
        x_meas,
        y_meas,
        psi_meas
    ):

        z = np.array([
            [x_meas],
            [y_meas],
            [psi_meas]
        ])

        y = z - self.H @ self.x

        S = (
            self.H @ self.P @ self.H.T
            + self.R
        )

        K = (
            self.P
            @ self.H.T
            @ np.linalg.inv(S)
        )

        self.x = self.x + K @ y

        I = np.eye(3)

        self.P = (
            I - K @ self.H
        ) @ self.P

        return (
            self.x[0,0],
            self.x[1,0],
            self.x[2,0]
        )