import numpy as np

from src.kf.KalmanFilterExtended import KalmanFilterExtended


class KalmanFilterExtendedImplicit(KalmanFilterExtended):
    def __init__(self,
                 x_size: int,
                 z_size: int,
                 ):
        super().__init__(x_size, z_size)
        self.J: np.array = np.array([])  # Jacobian matrix h respecto a Z

    def correct(self):
        self.update_h_jacobians()
        self.R = self.J @ self.R @ self.J.T
        self.update_K()
        self.X = self.Xp + self.K @ (-self.h)
        I = np.identity(6, dtype=np.float32)
        self.P = (I - self.K @ self.H) @ self.Pp
