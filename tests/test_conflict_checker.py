import unittest
from utils.input_loader import load_missions
from utils.conflict_checker import check_conflicts
from models.uncertainty_model import UncertaintyParameters


class TestConflictChecker(unittest.TestCase):

    def test_conflict_detection(self):
        missions = load_missions("input/missions.json")
        primary = missions[0]
        others = missions[1:]

        params = UncertaintyParameters(spatial_sigma=3.0, time_sigma=2.0)
        report = check_conflicts(primary, others, params, safety_distance=5.0)

        self.assertIn(report["status"], ["clear", "conflict_detected"])
        if report["status"] == "conflict_detected":
            self.assertTrue(len(report["conflicts"]) > 0)
            for conflict in report["conflicts"]:
                self.assertIn("conflicting_drone", conflict)
                self.assertIn("time", conflict)
                self.assertIn("spatial_distance", conflict)


if __name__ == '__main__':
    unittest.main()
