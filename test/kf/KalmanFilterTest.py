import unittest
from src.kf.KalmanFilterExtendedImplicit import KalmanFilterExtendedImplicit


class KalmanFilterTest(unittest.TestCase):
    def test_kalman_filter_extended_implicit(self):
        kalman_filter = KalmanFilterExtendedImplicit()
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
