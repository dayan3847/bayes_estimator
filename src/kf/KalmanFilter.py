import numpy as np


class KalmanFilter:
    def __init__(self,
                 x_size: int,
                 z_size: int,
                 ):
        self.x_size: int = x_size
        self.z_size: int = z_size

        # statePre
        self.Xp: np.array = np.array([])
        # statePost
        self.X: np.array = np.array([])
        # transitionMatrix
        self.A: np.array = np.array([])
        # measurementMatrix
        self.H: np.array = np.array([])
        # processNoiseCov
        self.Q: np.array = np.array([])
        # measurementNoiseCov
        self.R: np.array = np.array([])
        # errorCovPre
        self.Pp: np.array = np.array([])
        # gain
        self.K: np.array = np.array([])
        # errorCovPost
        self.P: np.array = np.array([])
        self.Z: np.array = np.array([])
        self.h: np.array = np.array([])

    def predict(self, dt: int):
        self.update_A(dt)
        self.Xp = self.A @ self.X
        self.Pp = self.A @ self.P @ self.A.T + self.Q

    def correct(self):
        pass

    def predict_correct(self, dt: int):
        self.predict(dt)
        self.correct()

    def init_X(self):
        pass

    def update_A(self, dt: int):
        pass
