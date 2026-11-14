# AI Crop-Safe Path Planning — Technical Documentation (Group 08)

**Role:** Member 6 — Testing, Visualisation, Performance Comparison, and Documentation

## 1. System Architecture & Task Coverage

This project implements an intelligent path-planning agent for an autonomous potato-field monitoring robot on a 2D grid. All six assignment tasks are covered across the notebook ([`CropSafe_PathPlanning.ipynb`](CropSafe_PathPlanning.ipynb)), formal specification ([`problem_formulation.md`](problem_formulation.md)), and evaluation suite ([`performance_evaluation.py`](performance_evaluation.py)):

| Task | Description | Implementation Location | Verification Status |
|---|---|---|---|
| **Task 1 — Field Environment** | 2D grid with safe cells, potato rows, 1-step crop-protection buffer, known/unknown blockades, start, and goal | `CropSafe_PathPlanning.ipynb` (Cells 2–15) | Verified via `show_input_grid` & `show_field` plots |
| **Task 2 — Problem Formulation** | State `(r, c)`, 4 orthogonal actions, transition model, goal test, unit & crop-safe path costs | `problem_formulation.md`, `PathPlanningProblem` (Cell 17) | Verified against notebook `PathPlanningProblem` class |
| **Task 3 — Search Algorithms** | BFS, DFS, Greedy Best-First Search (GBFS), and A\* Search with Manhattan heuristic `h(n)` | `CropSafe_PathPlanning.ipynb` (Cells 20, 27, 31) | All 4 algorithms reach `(11, 11)` on the test grid |
| **Task 4 — Crop-Safe Planning** | Impassable crop/protection cells plus graduated A\* penalties `{2: 6, 3: 2}` | `CROP_SAFETY_PENALTIES`, `crop_clearance`, `crop_safe_step_cost` (Cell 31) | Verified: A\* selects 22-move route with min clearance 4 |
| **Task 5 — Obstacle Detection & Replanning** | Hidden unknown blockades revealed before entry, map updated, route replanned | `simulate_with_discovery` (Cell 20), `simulate_a_star_crop_safe` (Cell 33) | Verified for DFS at `(0, 1)` and A\* at `(1, 2)` |
| **Task 6 — Performance Evaluation** | Path length, expanded nodes, execution time, goal reached, replanning count | [`performance_evaluation.py`](performance_evaluation.py) & [`README.md`](README.md) | All assertions passed; metrics recorded below |

## 2. Testing & Verification Methodology

Because the notebook's `show_robot_step` function includes `time.sleep(0.3)` and Matplotlib rendering per step for visual animation, measuring wall-clock time on the animated simulation would reflect rendering overhead rather than search efficiency.

To obtain accurate, reproducible metrics for **Task 6**, [`performance_evaluation.py`](performance_evaluation.py) mirrors the exact `PathPlanningProblem` and four search functions from `CropSafe_PathPlanning.ipynb` without animation delays and adds:

1. **Node Expansion Counters:** Incremented each time a state is popped and expanded by BFS, DFS, GBFS, or A\*.
2. **High-Resolution Timing:** `time.perf_counter()` averaged over 50 runs per algorithm/scenario.
3. **Automated Safety Assertions (`verify_path_safety`):**
   - Confirms every cell in the robot's traveled trajectory is not in `IMPASSABLE` (`{POTATO, PROTECTED, BLOCKADE, UNKNOWN}`).
   - Confirms every consecutive step has Manhattan distance `1` (strictly orthogonal movement, no teleportation or diagonal moves).
   - Confirms the final position equals `GOAL = (11, 11)`.

## 3. Experimental Scenarios & Results

All experiments use the shared 12×12 field: `START = (1, 1)`, `GOAL = (11, 11)`, potato rows at `(4, 3..7)` and `(8, 3..7)`, and known blockades at `(2, 7)`, `(6, 5)`, `(9, 9)`.

### Scenario 1: Baseline (No Unknown Blockades)

Matches the saved screenshots in `screenshots/` (`bfs_route.png`, `dfs_route.png`, `gbfs_route.png`, `astar_route.png`).

| Algorithm | Path length (moves) | Expanded nodes | Execution time (ms) | Destination reached? | Replanning operations | A\* crop-safe path cost | Min crop clearance |
|---|---:|---:|---:|---|---:|---:|---:|
| BFS | 20 | 107 | 0.439 | Yes | 0 | N/A | 2 steps |
| DFS | 30 | 34 | 0.136 | Yes | 0 | N/A | 2 steps |
| Greedy Best-First Search | 20 | 21 | 0.049 | Yes | 0 | N/A | 2 steps |
| A\* crop-safe | 22 | 37 | 0.300 | Yes | 0 | 22 | 4 steps |

### Scenario 2: Online Obstacle at `(0, 1)`

Matches the latest saved cell outputs in `CropSafe_PathPlanning.ipynb` (where DFS moves `Up` to `(0, 1)` first, detects the unknown blockade, and replans).

| Algorithm | Path length (moves) | Expanded nodes | Execution time (ms) | Destination reached? | Replanning operations | A\* crop-safe path cost | Min crop clearance |
|---|---:|---:|---:|---|---:|---:|---:|
| BFS | 20 | 107 | 0.408 | Yes | 0 | N/A | 2 steps |
| DFS | 24 | 61 | 0.252 | Yes | 1 | N/A | 2 steps |
| Greedy Best-First Search | 20 | 21 | 0.046 | Yes | 0 | N/A | 2 steps |
| A\* crop-safe | 22 | 37 | 0.273 | Yes | 0 | 22 | 4 steps |

### Scenario 3: Online Obstacle at `(1, 2)` (A\* Replanning)

Placed directly on A\*'s initial detour cell `(1, 2)` to verify Task 5 A\* online replanning.

| Algorithm | Path length (moves) | Expanded nodes | Execution time (ms) | Destination reached? | Replanning operations | A\* crop-safe path cost | Min crop clearance |
|---|---:|---:|---:|---|---:|---:|---:|
| BFS | 20 | 107 | 0.447 | Yes | 0 | N/A | 2 steps |
| DFS | 30 | 34 | 0.142 | Yes | 0 | N/A | 2 steps |
| Greedy Best-First Search | 20 | 21 | 0.048 | Yes | 0 | N/A | 2 steps |
| A\* crop-safe | 22 | 72 | 0.486 | Yes | 1 | 22 | 4 steps |

## 4. Key Analytical Observations

1. **Uninformed vs. Informed Search:** BFS guarantees the shortest unweighted path (20 moves) but explores uniformly in all directions, expanding 107 nodes. GBFS uses Manhattan distance to head directly toward `(11, 11)` and finds a 20-move path while expanding only 21 nodes.
2. **Why A\* Takes 22 Moves Instead of 20:** The 20-move corridor along column 1 passes cells at Manhattan distance 2 and 3 from the potato rows, which incur crop-proximity penalties (`+6` and `+2` per step). A\* routes around the top-right (`(1, 1) → (1, 2) → (0, 2) → ... → (0, 8) → ... → (11, 11)`), trading 2 extra moves for zero penalty (`cost = 22`) and a minimum crop clearance of **4 steps**.
3. **A\* report:** Notebook Cell 36 now matches the saved 12×12 run: 22 moves, crop-safe path cost 22, and 0 replans. The separate `screenshots/astar_route.png` image shows the A\* run with an unknown blockade at `(1, 2)`, which replans once.


