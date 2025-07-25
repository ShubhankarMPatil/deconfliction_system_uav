import unittest
from utils.input_loader import load_missions
from utils.visualizer_3d import plot_3d_uncertainty
from models.uncertainty_model import UncertaintyParameters


class TestVisualizer3D(unittest.TestCase):

    def test_plot_3d_uncertainty(self):
        missions = load_missions("input/missions.json")
        try:
            plot_3d_uncertainty(missions, primary_id="drone_1", uncertainty=UncertaintyParameters())
        except Exception as e:
            self.fail(f"3D plotting failed: {e}")


if __name__ == '__main__':
    unittest.main()
