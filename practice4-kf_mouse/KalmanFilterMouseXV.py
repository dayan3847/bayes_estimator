import numpy as np

from src.kf.KalmanFilter import KalmanFilter


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

    def predict(self, dt: int = 0):
        self.Xp = self.A @ self.X
        self.Pp = self.A @ self.P @ self.A.T + self.Q

    def init_X(self):
        x = self.Z[0]
        y = self.Z[1]
        self.X = np.array(
            [
                [x],  # x
                [y],  # y
                [0],  # dx
                [0],  # dy
            ]
        )


if __name__ == '__main__':
    kf = KalmanFilterMouseXV(1 / 60)
    kf.Z = np.array([0, 0, 0, 0], dtype=np.float32)
    kf.init_X()
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([1, 1, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([2, 2, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([3, 3, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([4, 4, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([5, 5, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
    kf.Z = np.array([6, 6, 0, 0], dtype=np.float32)
    kf.correct()
    print(kf.X)
    print(kf.P)
    kf.predict(1)
    print(kf.X)
    print(kf.P)
