import numpy as np

from src.kf.KalmanFilter import KalmanFilter


class KalmanFilterExtended(KalmanFilter):
    def correct(self):
        self.update_h_jacobians()
        self.update_K()
        self.X = self.Xp + self.K @ (self.Z - self.h)
        I = np.identity(6, dtype=np.float32)
        self.P = (I - self.K @ self.H) @ self.Pp

    def update_h_jacobians(self):
        pass

    def update_K(self):
        Ht = self.H.T
        HxPpxHtmR = self.H @ self.Pp @ Ht + self.R
        inv_HxPpxHtmR = np.linalg.inv(HxPpxHtmR)
        self.K = self.Pp @ Ht @ inv_HxPpxHtmR
