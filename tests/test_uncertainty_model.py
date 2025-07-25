import unittest
import numpy as np
from utils.input_loader import load_missions
from models.uncertainty_model import generate_uncertainty_tube, UncertaintyParameters, UncertainPosition

class TestUncertaintyModel(unittest.TestCase):

    def test_uncertainty_tube_length_and_structure(self):
        missions = load_missions("input/missions.json")
        mission = missions[0]
        tube = generate_uncertainty_tube(mission, UncertaintyParameters())

        self.assertEqual(len(tube), len(mission.waypoints))
        self.assertTrue(all(isinstance(p, UncertainPosition) for p in tube))

        for pos in tube:
            self.assertEqual(len(pos.mean), 4)
            self.assertEqual(pos.cov.shape, (4, 4))
            self.assertAlmostEqual(np.trace(pos.cov), 9 + 9 + 9 + 4)  # 3m sigma^2 + 2s sigma^2

if __name__ == '__main__':
    unittest.main()
