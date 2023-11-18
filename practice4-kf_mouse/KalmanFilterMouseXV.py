import numpy as np

from src.kf.KalmanFilter import KalmanFilter
from MouseTracker import MouseTracker


class KalmanFilterMouseXV(KalmanFilter):
    def __init__(self, dt: float):
        super().__init__(4, 4)
        self.Q = np.identity(4, dtype=np.float32) * 1e-4
        self.R = np.identity(4, dtype=np.float32) * 10
        self.P = np.identity(4, dtype=np.float32) * .1
        self.H = np.identity(4, dtype=np.float32)
        self.A = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
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
        ])


if __name__ == '__main__':
    window_name: str = 'Mouse Tracker [Position,Velocity]'

    # delta time
    dt: float = 1 / 60
    kf = KalmanFilterMouseXV(dt)
    # Measurement
    kf.Z = np.array([0, 0, 0, 0])
    kf.init_X()

    tracker = MouseTracker(kf, dt, window_name)
    tracker.run()
