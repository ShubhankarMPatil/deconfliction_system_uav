import unittest
from utils.input_loader import load_missions
from utils.heatmap_generator import generate_heatmap
from models.uncertainty_model import UncertaintyParameters


class TestHeatmapGenerator(unittest.TestCase):

    def test_heatmap_generation(self):
        missions = load_missions("input/missions.json")
        bounds = (150, 150, 30)
        time_range = (0, 150)

        heatmap = generate_heatmap(missions, bounds, time_range, UncertaintyParameters())

        self.assertIsInstance(heatmap, dict)
        self.assertTrue(len(heatmap) > 0)

        for key, value in heatmap.items():
            self.assertEqual(len(key), 4)
            self.assertTrue(isinstance(value, int))

        print(f"Generated heatmap with {len(heatmap)} active cells.")

if __name__ == '__main__':
    unittest.main()
