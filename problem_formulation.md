# Problem Formulation: Crop-Safe Field Path Planning

## Environment

The field is a finite two-dimensional grid with `ROWS` rows and `COLS` columns. A location is represented as `(row, column)`, with `(0, 0)` at the top-left. The configured map can contain:

- Safe travel cells
- Potato crop cells
- Crop-protection cells, automatically placed one Manhattan step from crop cells
- Known blockades
- Initially unknown blockades, which are present in the true field but omitted from the robot’s initial map
- A start cell and a goal cell

Blockades have the highest priority when the field is constructed. If a blockade overlaps a crop or protection location, that cell is treated as a blockade. Start and goal must be on traversable cells.

## State

A state is the robot’s current grid position:

```text
(row, column)
```

The initial state is the configured start coordinate.

## Actions

The robot can move one cell in one of four directions:

- Up: `(-1, 0)`
- Down: `(1, 0)`
- Left: `(0, -1)`
- Right: `(0, 1)`

Diagonal moves are not allowed. A move is legal only if the destination remains inside the grid and is not known to be a crop, protection, or blockade cell.

## Transition model

For a state `(r, c)` and a legal action with offset `(dr, dc)`, the next state is:

```text
(r + dr, c + dc)
```

The transition is rejected if the action is illegal or if the destination is outside the field or impassable on the robot’s current map.

## Goal test

The goal test succeeds when the robot’s current coordinate equals the configured goal coordinate.

## Path cost

- **Standard search (BFS, DFS, GBFS):** One unit per legal move (`cost = 1`), so the path cost equals the number of moves.
- **Crop-safe A\* search:** Each legal move has a base cost of `1` plus a graduated proximity penalty for safe cells close to potato crops (`+6` at Manhattan distance 2 from a crop, `+2` at Manhattan distance 3, `0` at distance 4 or greater). Crop cells, protection cells, and known blockades remain strictly impassable.

## Unknown-blockade discovery and replanning

The true field may contain unknown blockades that the robot’s initial map does not show. Each planner first computes a route from the robot’s current position using its known map. If the next planned cell contains an unknown blockade in the true field, the robot detects it before entering, marks it as a known blockade in its map, remains in its current cell, and replans a new route to the goal (using A\* for crop-safe online replanning, or the active solver in algorithm-specific simulations). The robot repeats this process until it reaches the goal or no feasible route exists.

## Search algorithms in the notebook

- **Breadth-First Search (BFS):** explores the grid by increasing number of moves and returns a shortest path when every move has equal cost.
- **Depth-First Search (DFS):** explores one branch deeply before backtracking. It may find a longer path than BFS and can have different behavior as action ordering changes.
- **Greedy Best-First Search (GBFS):** uses the Manhattan distance heuristic `h(n) = |r - r_g| + |c - c_g|` to expand the cell that appears closest to the goal.
- **A\* Crop-Safe Search:** minimizes `f(n) = g(n) + h(n)`, combining accumulated crop-safe movement cost `g(n)` with the admissible Manhattan distance heuristic `h(n)` to balance path length and crop clearance.
