import unittest
from utils.input_loader import load_missions
from utils.visualizer_2d import plot_topdown_2d


class TestVisualizer2D(unittest.TestCase):

    def test_plot_topdown(self):
        missions = load_missions("input/missions.json")
        try:
            plot_topdown_2d(missions, primary_id="drone_1", safety_radius=10.0)
        except Exception as e:
            self.fail(f"Plotting failed with error: {e}")


if __name__ == '__main__':
    unittest.main()
