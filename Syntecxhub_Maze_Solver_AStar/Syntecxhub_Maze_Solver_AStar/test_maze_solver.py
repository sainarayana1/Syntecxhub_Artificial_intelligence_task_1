import unittest
from maze_solver import Maze, astar


class TestAStar(unittest.TestCase):
    def test_straight_line(self):
        r = astar(Maze("S...G"))
        self.assertTrue(r.found)
        self.assertEqual(r.cost, 4)

    def test_shortest_path_around_wall(self):
        m = Maze("S.#..\n..#.G\n.....")
        for h in ("manhattan", "euclidean"):
            r = astar(m, h)
            self.assertEqual(r.path[0], m.start)
            self.assertEqual(r.path[-1], m.goal)

    def test_both_heuristics_same_cost(self):
        m = Maze.from_file("mazes/simple.txt")
        self.assertEqual(astar(m, "manhattan").cost, astar(m, "euclidean").cost)

    def test_unreachable(self):
        r = astar(Maze.from_file("mazes/unreachable.txt"))
        self.assertFalse(r.found)
        self.assertIsNone(r.path)

    def test_path_has_no_walls_and_valid_steps(self):
        m = Maze.from_file("mazes/simple.txt")
        r = astar(m)
        for a, b in zip(r.path, r.path[1:]):
            self.assertEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), 1)
            self.assertTrue(m.is_open(b))

    def test_missing_start_or_goal(self):
        with self.assertRaises(ValueError):
            Maze("....G")


if __name__ == "__main__":
    unittest.main()
