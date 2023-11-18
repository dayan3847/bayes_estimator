import numpy as np

from src.kf.KalmanFilter import KalmanFilter


class KalmanFilterMouseXV(KalmanFilter):
    def __init__(self):
        super().__init__(4, 4)
        self.Q = np.identity(4, dtype=np.float32) * 1e-4
        self.R = np.identity(4, dtype=np.float32) * 10
        self.P = np.identity(4, dtype=np.float32) * .1

    def predict(self, dt: int):
        self.update_A(dt)
        self.Xp = self.A @ self.X
        self.Pp = self.A @ self.P @ self.A.T + self.Q

    def correct(self):
        self.update_h_jacobians()
        self.update_K()
        self.X = self.Xp + self.K @ (self.Z - self.h)
        I = np.identity(4, dtype=np.float32)
        self.P = (I - self.K @ self.H) @ self.Pp

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

    def update_A(self, dt: int):
        self.A = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )


if __name__ == '__main__':
    kf = KalmanFilterMouseXV()
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
