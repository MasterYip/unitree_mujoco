import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_follow import CameraFollower


def test_translation_yaw_smoothing_and_reset():
    q_yaw_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    follower = CameraFollower(offset=(1.0, 0.0, 0.2), azimuth=90.0)
    lookat, azimuth = follower.update((2.0, 3.0, 0.8), q_yaw_90, 0.02)
    np.testing.assert_allclose(lookat, (3.0, 3.0, 1.0), atol=1e-12)
    assert azimuth == 90.0

    follower = CameraFollower(offset=(1.0, 0.0, 0.2), azimuth=90.0, follow_yaw=True)
    lookat, azimuth = follower.update((2.0, 3.0, 0.8), q_yaw_90, 0.02)
    np.testing.assert_allclose(lookat, (2.0, 4.0, 1.0), atol=1e-12)
    assert abs(azimuth - 180.0) < 1e-12

    follower = CameraFollower(smoothing_tau=1.0)
    follower.update((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), 0.1)
    lookat, _ = follower.update((10.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), 0.1)
    assert 0.0 < lookat[0] < 10.0
    follower.reset()
    lookat, _ = follower.update((10.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), 0.1)
    assert lookat[0] == 10.0
