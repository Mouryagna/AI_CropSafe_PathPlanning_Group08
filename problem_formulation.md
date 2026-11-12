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

The current basic path cost is one unit per legal move. Therefore, the path cost is the number of moves. Crop cells, protection cells, and known blockades cannot be traversed, so paths avoid them entirely.

## Unknown-blockade discovery and replanning

The true field contains unknown blockades that the robot’s initial map does not show. BFS and DFS first plan from the robot’s current position using that map. If the next planned cell contains an unknown blockade in the true field, the robot detects it before entering, marks it as blocked in its map, remains in its current cell, and replans to the goal using the same search algorithm. The robot repeats this process until it reaches the goal or no route exists.

## Search algorithms in the notebook

- **Breadth-First Search (BFS):** explores the grid by increasing number of moves and returns a shortest path when every move has equal cost.
- **Depth-First Search (DFS):** explores one branch deeply before backtracking. It may find a longer path than BFS and can have different behavior as action ordering changes.
