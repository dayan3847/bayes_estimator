import numpy as np

from src.kf.KalmanFilter import KalmanFilter
from MouseTracker import MouseTracker


class KalmanFilterMouseXVA(KalmanFilter):
    def __init__(self, dt: float):
        super().__init__(6, 6)
        self.Q = np.identity(6, dtype=np.float32) * 1e-4
        self.R = np.identity(6, dtype=np.float32) * 10
        self.P = np.identity(6, dtype=np.float32) * .1
        self.H = np.identity(6, dtype=np.float32)
        ddt = (dt ** 2) / 2
        self.A = np.array(
            [
                [1, 0, dt, 0, ddt, 0],
                [0, 1, 0, dt, 0, ddt],
                [0, 0, 1, 0, dt, 0],
                [0, 0, 0, 1, 0, dt],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

    def init_X(self):
        x = self.Z[0]
        y = self.Z[1]
        self.X = np.array([
            x,  # x
            y,  # y
            0,  # dx
            0,  # dy
            0,  # ddx
            0,  # ddy
        ])


if __name__ == '__main__':
    window_name: str = 'Mouse Tracker [Position,Velocity,Acceleration]'

    # delta time
    dt: float = 1 / 60
    kf = KalmanFilterMouseXVA(dt)
    # Measurement
    kf.Z = np.array([0, 0, 0, 0, 0, 0])
    kf.init_X()

    tracker = MouseTracker(kf, dt, window_name, degree=2)
    tracker.run()
