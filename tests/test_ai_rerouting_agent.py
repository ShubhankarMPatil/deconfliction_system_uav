import unittest
from utils.input_loader import load_missions
from models.ai_rerouting_agent import reroute_path
from models.uncertainty_model import UncertaintyParameters


class TestAIReroutingAgent(unittest.TestCase):

    def test_rerouting_succeeds(self):
        missions = load_missions("input/missions.json")
        primary = missions[0]
        others = missions[1:]

        bounds = (150, 150, 30)  # X, Y, Z in meters
        uncertainty = UncertaintyParameters()

        new_path = reroute_path(primary, others, bounds, uncertainty)

        self.assertTrue(isinstance(new_path, list))
        if new_path:
            self.assertTrue(all(hasattr(wp, 'x') for wp in new_path))
            print(f"Suggested reroute has {len(new_path)} waypoints.")

if __name__ == '__main__':
    unittest.main()
