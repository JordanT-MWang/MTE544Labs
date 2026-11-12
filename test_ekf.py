"""ROS-free EKF math self-check. Run: python3 test_ekf.py"""
import numpy as np
from kalman_filter import kalman_filter


def main():
    kf = kalman_filter(
        0.5 * np.eye(6), 0.5 * np.eye(6), 0.5 * np.eye(4),
        np.zeros(6), 0.1,
    )
    for _ in range(100):
        kf.predict()
        kf.update([0.2, 0.1, 0.0, 0.02])
        assert np.all(np.isfinite(kf.x))
        assert np.all(np.isfinite(kf.P))
        assert np.min(np.linalg.eigvalsh(kf.P)) > -1e-9
    assert kf.x[0] > 0.1, "Estimated robot did not move forward"
    assert abs(kf.x[3] - 0.1) < 0.05, "Estimated yaw rate is implausible"
    assert abs(kf.x[4] - 0.2) < 0.05, "Estimated velocity is implausible"
    print("PASS: 100 EKF predict/update cycles and covariance checks")
    print("Final state [x,y,theta,w,v,vdot] =", np.round(kf.x, 4))


if __name__ == "__main__":
    main()
