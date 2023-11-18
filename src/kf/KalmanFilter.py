import numpy as np


class KalmanFilter:
    def __init__(self,
                 x_size: int,
                 z_size: int,
                 ):
        self.x_size: int = x_size
        self.z_size: int = z_size

        # statePre
        self.Xp: np.array = None
        # statePost
        self.X: np.array = None
        # transitionMatrix
        self.A: np.array = None
        # measurementMatrix
        self.H: np.array = None
        # processNoiseCov
        self.Q: np.array = None
        # measurementNoiseCov
        self.R: np.array = None
        # errorCovPre
        self.Pp: np.array = None
        # gain
        self.K: np.array = None
        # errorCovPost
        self.P: np.array = None
        self.Z: np.array = None
        self.h: np.array = None

    def predict(self, dt: int):
        self.update_A(dt)
        self.Xp = self.A @ self.X
        self.Pp = self.A @ self.P @ self.A.T + self.Q

    def predict_correct(self, dt: int):
        self.predict(dt)
        self.correct()

    def init_X(self):
        pass

    def update_A(self, dt: int):
        pass

    def update_K(self):
        Ht = self.H.T
        HxPpxHtmR = self.H @ self.Pp @ Ht + self.R
        inv_HxPpxHtmR = np.linalg.inv(HxPpxHtmR)
        self.K = self.Pp @ Ht @ inv_HxPpxHtmR

    def correct(self):
        self.update_h_jacobians()
        self.update_K()
        self.X = self.Xp + self.K @ (self.Z - self.h)
        I = np.identity(self.x_size, dtype=np.float32)
        self.P = (I - self.K @ self.H) @ self.Pp

    def update_h_jacobians(self):
        self.h = self.H @ self.Xp
