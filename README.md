# Syntecxhub_Maze_Solver_AStar

**Syntecxhub AI Internship – Week 1, Project 1: Maze Solver using A\* Search**

## Features
- Maze/grid model: `#` wall, `.` open, `S` start, `G` goal. Each open cell is a node; edges connect 4-directional neighbours (cost 1).
- A\* search (`f = g + h`) using a priority queue (`heapq`).
- Selectable heuristic: **Manhattan** or **Euclidean** (both admissible, so the path is optimal).
- Returns the shortest path, or reports cleanly when the goal is **unreachable**.
- Visualisation: console output (`*` path, `·` explored) and optional matplotlib plot.
- Unit tests.

## Run
```bash
python maze_solver.py                              # built-in maze
python maze_solver.py mazes/simple.txt             # your own maze file
python maze_solver.py mazes/unreachable.txt -H euclidean
python maze_solver.py --plot                       # needs: pip install matplotlib
python -m unittest -v                              # run tests
```

## Sample output
```
####################
#S*****#***********#
#·####*#*#########*#
...
Path found!  Length: 39 steps | Nodes explored: 91 | Heuristic: manhattan
```

## How it works
1. Push the start node with `f = h(start)`.
2. Pop the node with the lowest `f`, mark it explored.
3. If it is the goal, rebuild the path through `came_from`.
4. Otherwise relax each neighbour: if `g + 1` improves its best-known cost, record it and push it.
5. If the queue empties, the goal cannot be reached.

## Project structure
```
maze_solver.py        # Maze, A*, heuristics, visualisation, CLI
test_maze_solver.py   # unit tests
mazes/                # sample mazes (solvable + unreachable)
requirements.txt
```
<img width="560" height="331" alt="image" src="https://github.com/user-attachments/assets/54f4cf42-0ed7-4ac0-a96f-c5ba7f7f2ffb" />
