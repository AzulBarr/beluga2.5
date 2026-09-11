import math
import unittest

from body_frame_offset import residuals, summarize, transform
from evaluate_trajectory import Pose


def circle(n=200, radius=4.0):
    """A closed arc: yaw sweeps a full turn, so a body-frame offset cannot be
    mistaken for a world-frame translation."""
    poses = []
    for i in range(n):
        a = 2 * math.pi * i / n
        poses.append(Pose(i * 100_000_000, radius * math.cos(a), radius * math.sin(a), a))
    return poses


class BodyFrameOffsetTests(unittest.TestCase):
    def test_transform_moves_along_the_body_axes(self):
        p = Pose(0, 1.0, 2.0, math.pi / 2)
        moved = transform([p], 0.5, 0.0, 0.0)[0]
        # Facing +y, so a forward lever displaces y, not x.
        self.assertAlmostEqual(moved.x, 1.0)
        self.assertAlmostEqual(moved.y, 2.5)

    def test_diagnosis_recovers_an_injected_lever(self):
        reference = circle()
        estimate = transform(reference, 0.24, -0.05, 0.01)
        summary = summarize(reference, estimate)
        self.assertAlmostEqual(summary['mean_forward_m'], 0.24, places=6)
        self.assertAlmostEqual(summary['mean_lateral_m'], -0.05, places=6)
        self.assertAlmostEqual(summary['mean_yaw_rad'], 0.01, places=6)
        # A pure lever arm is perfectly constant in the body frame.
        self.assertLess(summary['stdev_forward_m'], 1e-9)
        self.assertLess(summary['stdev_lateral_m'], 1e-9)

    def test_applying_the_measured_lever_removes_the_offset(self):
        reference = circle()
        estimate = transform(reference, 0.24, 0.0, 0.0)
        corrected = transform(estimate, -0.24, 0.0, 0.0)
        self.assertLess(summarize(reference, corrected)['position_rmse_m'], 1e-9)

    def test_a_world_translation_is_not_reported_as_a_lever(self):
        """Shifting the whole trajectory in the world leaves a body-frame error
        that rotates with the robot, so its mean cancels over a closed loop."""
        reference = circle()
        estimate = [Pose(p.stamp, p.x + 0.24, p.y, p.yaw) for p in reference]
        summary = summarize(reference, estimate)
        self.assertLess(summary['offset_magnitude_m'], 1e-9)
        self.assertGreater(summary['position_rmse_m'], 0.2)

    def test_centred_rmse_never_exceeds_the_raw_rmse(self):
        reference = circle()
        estimate = transform(reference, 0.24, 0.0, 0.0)
        summary = summarize(reference, estimate)
        self.assertLessEqual(summary['position_rmse_without_mean_offset_m'],
                             summary['position_rmse_m'] + 1e-12)

    def test_residuals_use_the_reference_heading(self):
        reference = [Pose(0, 0.0, 0.0, math.pi)]
        estimate = [Pose(0, -1.0, 0.0, math.pi)]
        forward, lateral, _ = residuals(reference, estimate)
        # Facing -x, an estimate at -1 is one metre AHEAD.
        self.assertAlmostEqual(forward[0], 1.0)
        self.assertAlmostEqual(lateral[0], 0.0)


if __name__ == '__main__':
    unittest.main()
