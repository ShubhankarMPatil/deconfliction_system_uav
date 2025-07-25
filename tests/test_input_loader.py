import unittest
from utils.input_loader import load_missions, DroneMission


class TestInputLoader(unittest.TestCase):

    def test_load_missions(self):
        missions = load_missions("input/missions.json")
        self.assertEqual(len(missions), 2)

        mission1 = missions[0]
        self.assertIsInstance(mission1, DroneMission)
        self.assertEqual(mission1.drone_id, "drone_1")
        self.assertEqual(len(mission1.waypoints), 3)
        self.assertEqual(mission1.priority, "normal")
        self.assertEqual(mission1.get_path()[1], (100, 0, 10, 50))

        mission2 = missions[1]
        self.assertEqual(mission2.priority, "emergency")
        self.assertEqual(mission2.get_path()[-1], (100, 0, 10, 120))


if __name__ == '__main__':
    unittest.main()
