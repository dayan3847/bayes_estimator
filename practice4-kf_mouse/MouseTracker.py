import numpy as np
import cv2
import time

from src.kf.KalmanFilter import KalmanFilter


class MouseTracker:
    def __init__(self, kf: KalmanFilter, dt: float, window_name: str, degree=1):
        self.kf: KalmanFilter = kf
        self.dt: float = dt
        self.window_name: str = window_name
        self.degree: int = degree

        self.points_Z: list[tuple[int, int]] = [(int(kf.Z[0]), int(kf.Z[1])), (int(kf.Z[0]), int(kf.Z[1]))]
        self.points_X: list[tuple[int, int]] = [(int(kf.X[0]), int(kf.X[1]))]

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            point_Z_k_1 = self.points_Z[-1]
            dx = (x - point_Z_k_1[0])
            xp = dx / self.dt
            dy = (y - point_Z_k_1[1])
            yp = dy / self.dt
            new_Z = [x, y, xp, yp]
            if self.degree == 2:
                point_Z_k_2 = self.points_Z[-2]
                ddx = (dx - (point_Z_k_1[0] - point_Z_k_2[0])) / self.dt
                ddy = (dy - (point_Z_k_1[1] - point_Z_k_2[1])) / self.dt
                new_Z.append(ddx)
                new_Z.append(ddy)

            self.kf.Z = np.array(new_Z)
            new_point_Z = (int(self.kf.Z[0]), int(self.kf.Z[1]))
            if not np.array_equal(point_Z_k_1, new_point_Z):
                self.points_Z.append(new_point_Z)

    def draw_points(self):
        # Clear image
        img = np.zeros((600, 800, 3), dtype=np.uint8)
        # Draw points
        for points, color in zip([self.points_Z, self.points_X], [(0, 0, 255), (255, 0, 0)]):
            for i in range(1, len(points)):
                cv2.line(img, points[i - 1], points[i], color, 1, cv2.LINE_AA, 0)

        # Show image
        cv2.imshow(self.window_name, img)

    def run(self):
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        running: bool = True
        while running:
            self.kf.predict_correct()

            new_point_X = (int(self.kf.X[0]), int(self.kf.X[1]))
            if not np.array_equal(self.points_X[-1], new_point_X):
                self.points_X.append(new_point_X)

            self.draw_points()

            # sleep
            time.sleep(self.dt)
            # q to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
        cv2.destroyAllWindows()
