"""
Task 6: Testing & Performance Evaluation for AI Crop-Safe Path Planning (Group 08).

Evaluates BFS, DFS, Greedy Best-First Search (GBFS), and A* Crop-Safe Search
across baseline and online-obstacle scenarios, verifying crop-safety constraints
and reporting path length, expanded nodes, execution time, goal reachability,
and replanning operations.
"""

from collections import deque
from dataclasses import dataclass
from itertools import count
import heapq
import time

# Cell states matching CropSafe_PathPlanning.ipynb
SAFE, POTATO, PROTECTED, BLOCKADE, START_CELL, GOAL_CELL, UNKNOWN = range(7)
MOVES = {"Up": (-1, 0), "Down": (1, 0), "Left": (0, -1), "Right": (0, 1)}
IMPASSABLE = {POTATO, PROTECTED, BLOCKADE, UNKNOWN}
CROP_SAFETY_PENALTIES = {2: 6, 3: 2}

# Shared 12x12 demonstration field configuration
ROWS, COLS = 12, 12
START = (1, 1)
GOAL = (11, 11)
KNOWN_BLOCKADES = [(2, 7), (6, 5), (9, 9)]
CROP_CELLS = [(4, c) for c in range(3, 8)] + [(8, c) for c in range(3, 8)]


def build_field(unknown_blockades):
    """Build true_field and robot's known_field for the shared 12x12 grid."""
    grid = [[SAFE for _ in range(COLS)] for _ in range(ROWS)]
    crop_set = set(CROP_CELLS)
    for r, c in crop_set:
        grid[r][c] = POTATO
    for cr, cc in crop_set:
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = cr + dr, cc + dc
            if 0 <= r < ROWS and 0 <= c < COLS and (r, c) not in crop_set:
                grid[r][c] = PROTECTED
    for r, c in KNOWN_BLOCKADES:
        grid[r][c] = BLOCKADE
    grid[START[0]][START[1]] = START_CELL
    grid[GOAL[0]][GOAL[1]] = GOAL_CELL

    known_field = [row.copy() for row in grid]
    true_field = [row.copy() for row in grid]
    for r, c in unknown_blockades:
        true_field[r][c] = UNKNOWN
    return true_field, known_field


@dataclass
class PathPlanningProblem:
    grid: list
    start: tuple
    goal: tuple

    def actions(self, state):
        result = []
        for action, (dr, dc) in MOVES.items():
            nr, nc = state[0] + dr, state[1] + dc
            if (
                0 <= nr < len(self.grid)
                and 0 <= nc < len(self.grid[0])
                and self.grid[nr][nc] not in IMPASSABLE
            ):
                result.append(action)
        return result

    def transition(self, state, action):
        dr, dc = MOVES[action]
        return state[0] + dr, state[1] + dc

    def goal_test(self, state):
        return state == self.goal


def reconstruct_path(parent, goal):
    path, state = [], goal
    while state is not None:
        path.append(state)
        state = parent[state]
    return list(reversed(path))


def manhattan_distance(state, goal):
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])


def crop_clearance(cell):
    return min(abs(cell[0] - cr) + abs(cell[1] - cc) for cr, cc in CROP_CELLS)


def crop_safe_step_cost(cell):
    return 1 + CROP_SAFETY_PENALTIES.get(crop_clearance(cell), 0)


def bfs_search(problem):
    expanded = 0
    frontier = deque([problem.start])
    parent = {problem.start: None}
    while frontier:
        state = frontier.popleft()
        expanded += 1
        if problem.goal_test(state):
            return reconstruct_path(parent, state), expanded
        for action in MOVES:
            if action in problem.actions(state):
                nxt = problem.transition(state, action)
                if nxt not in parent:
                    parent[nxt] = state
                    frontier.append(nxt)
    return None, expanded


def dfs_search(problem):
    expanded = 0
    frontier = [problem.start]
    parent = {problem.start: None}
    while frontier:
        state = frontier.pop()
        expanded += 1
        if problem.goal_test(state):
            return reconstruct_path(parent, state), expanded
        for action in reversed(list(MOVES)):
            if action in problem.actions(state):
                nxt = problem.transition(state, action)
                if nxt not in parent:
                    parent[nxt] = state
                    frontier.append(nxt)
    return None, expanded


def gbfs_search(problem):
    expanded = 0
    frontier, tie_breaker = [], count()
    start, goal = problem.start, problem.goal
    heapq.heappush(frontier, (manhattan_distance(start, goal), next(tie_breaker), start))
    parent = {start: None}
    explored = set()
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in explored:
            continue
        explored.add(current)
        expanded += 1
        if problem.goal_test(current):
            return reconstruct_path(parent, current), expanded
        for action in problem.actions(current):
            nxt = problem.transition(current, action)
            if nxt not in explored and nxt not in parent:
                parent[nxt] = current
                heapq.heappush(
                    frontier, (manhattan_distance(nxt, goal), next(tie_breaker), nxt)
                )
    return None, expanded


def a_star_crop_safe_search(problem):
    expanded = 0
    frontier, tie_breaker = [], count()
    start = problem.start
    heapq.heappush(frontier, (manhattan_distance(start, problem.goal), next(tie_breaker), start))
    parent = {start: None}
    g_score = {start: 0}
    while frontier:
        f_score, _, current = heapq.heappop(frontier)
        if f_score != g_score[current] + manhattan_distance(current, problem.goal):
            continue
        expanded += 1
        if problem.goal_test(current):
            return reconstruct_path(parent, current), expanded
        for action in problem.actions(current):
            nxt = problem.transition(current, action)
            tentative_g = g_score[current] + crop_safe_step_cost(nxt)
            if tentative_g < g_score.get(nxt, float("inf")):
                parent[nxt] = current
                g_score[nxt] = tentative_g
                priority = tentative_g + manhattan_distance(nxt, problem.goal)
                heapq.heappush(frontier, (priority, next(tie_breaker), nxt))
    return None, expanded


def simulate_online(search_fn, true_field, known_field, is_astar=False):
    robot_map = [row.copy() for row in known_field]
    current = START
    traveled = [current]
    steps = replans = total_cost = total_expanded = 0

    while current != GOAL:
        route, expanded = search_fn(PathPlanningProblem(robot_map, current, GOAL))
        total_expanded += expanded
        if route is None:
            return {
                "reached": False,
                "moves": steps,
                "expanded": total_expanded,
                "replans": replans,
                "cost": total_cost if is_astar else None,
                "traveled": traveled,
            }

        detected = False
        for nxt in route[1:]:
            r, c = nxt
            if true_field[r][c] == UNKNOWN:
                robot_map[r][c] = BLOCKADE
                replans += 1
                detected = True
                break
            current = nxt
            traveled.append(current)
            steps += 1
            if is_astar:
                total_cost += crop_safe_step_cost(current)
            if current == GOAL:
                break
        if not detected and current != GOAL:
            break

    return {
        "reached": current == GOAL,
        "moves": steps,
        "expanded": total_expanded,
        "replans": replans,
        "cost": total_cost if is_astar else None,
        "traveled": traveled,
    }


def verify_path_safety(traveled, true_field):
    """Ensure the robot never entered a potato, crop-protection, or blockade cell."""
    for r, c in traveled:
        assert true_field[r][c] not in IMPASSABLE, (
            f"Safety violation at {(r, c)} with cell type {true_field[r][c]}"
        )
    for a, b in zip(traveled, traveled[1:]):
        dist = abs(a[0] - b[0]) + abs(a[1] - b[1])
        assert dist == 1, f"Illegal move from {a} to {b} (Manhattan distance {dist})"


def evaluate_scenario(title, unknown_blockades, repeats=50):
    true_field, known_field = build_field(unknown_blockades)
    algorithms = [
        ("BFS", bfs_search, False),
        ("DFS", dfs_search, False),
        ("Greedy Best-First Search", gbfs_search, False),
        ("A* crop-safe", a_star_crop_safe_search, True),
    ]
    print(f"\n=== {title} ===")
    print(
        "| Algorithm | Path length (moves) | Expanded nodes | Execution time (ms) | "
        "Destination reached? | Replanning operations | A* crop-safe path cost | Min crop clearance |"
    )
    print("|---|---:|---:|---:|---|---:|---:|---:|")
    for name, fn, is_astar in algorithms:
        res = simulate_online(fn, true_field, known_field, is_astar)
        verify_path_safety(res["traveled"], true_field)
        assert res["reached"], f"{name} failed to reach {GOAL}"

        t0 = time.perf_counter()
        for _ in range(repeats):
            simulate_online(fn, true_field, known_field, is_astar)
        avg_ms = (time.perf_counter() - t0) * 1000 / repeats

        min_clr = min(crop_clearance(c) for c in res["traveled"] if c not in (START, GOAL))
        cost_str = str(res["cost"]) if is_astar else "N/A"
        print(
            f"| {name} | {res['moves']} | {res['expanded']} | {avg_ms:.3f} | "
            f"{'Yes' if res['reached'] else 'No'} | {res['replans']} | {cost_str} | {min_clr} |"
        )


if __name__ == "__main__":
    evaluate_scenario("Scenario 1: Baseline (no unknown blockades)", [])
    evaluate_scenario("Scenario 2: Online obstacle at (0, 1)", [(0, 1)])
    evaluate_scenario("Scenario 3: Online obstacle at (1, 2) (A* replanning)", [(1, 2)])
    print("\nAll path-safety and reachability assertions passed.")

