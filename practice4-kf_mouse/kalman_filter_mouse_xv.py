import numpy as np
import cv2
import time

from KalmanFilterMouseXV import KalmanFilterMouseXV

# delta time
dt: float = 1 / 60
kf = KalmanFilterMouseXV(dt)


def mouse_callback(event: int, x: int, y: int, flags: int, param) -> None:
    if event == cv2.EVENT_MOUSEMOVE:
        global kf
        kf.Z = np.array([x, y], dtype=np.float32)


if __name__ == '__main__':

    window_name: str = 'Mouse Tracker'
    img = np.zeros((600, 800, 3), dtype=np.uint8)

    # Measurement
    kf.Z = np.array([0, 0])
    kf.init_X()

    # Vector of points Measurements
    points: np.ndarray = np.array([kf.Z.astype(np.int32)], dtype=np.int32)
    # Vector of points Kalman
    points_kalman: np.ndarray = np.array([kf.Z.astype(np.int32)], dtype=np.int32)

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    running: bool = True
    while running:
        # Predict
        kf.predict()
        # Correct
        kf.correct()
        # Get the estimated position
        estimated_position: np.ndarray = kf.X.T[0][:2].astype(np.int32)

        points = np.append(points, [kf.Z], axis=0)
        points_kalman = np.append(points_kalman, [estimated_position], axis=0)

        # Draw
        if not np.array_equal(points[-2], points[-1]):
            cv2.line(img, tuple(points[-2]), tuple(points[-1]), (0, 0, 255), 1, cv2.LINE_AA, 0)
        if not np.array_equal(points_kalman[-2], points_kalman[-1]):
            cv2.line(img, tuple(points_kalman[-2]), tuple(points_kalman[-1]), (255, 0, 0), 1,
                     cv2.LINE_AA, 0)

        # Show image
        cv2.imshow(window_name, img)
        # sleep
        time.sleep(dt)
        # q to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False
    cv2.destroyAllWindows()
