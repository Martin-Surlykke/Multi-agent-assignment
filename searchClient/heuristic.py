from abc import ABC, abstractmethod
from collections import deque
import sys

from searchclient.state import State

class Heuristic(ABC):
    def __init__(self, initial_state: State) -> None:
        # Pre-process the static parts of the level (BOX heat map).
        rows = len(State.walls)
        cols = len(State.walls[0]) if rows > 0 else 0
        self.inf = 10**7

        self.box_heat_map = [[[self.inf for _ in range(cols)] for _ in range(rows)] for _ in range(26)]
        self.box_in_level = [False for _ in range(26)]

        # Find box goals and start BFS from each
        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                goal = State.goals[r][c]
                if "A" <= goal <= "Z":
                    box_id = ord(goal) - ord("A")
                    self.box_in_level[box_id] = True
                    # BFS to fill heatmap for this box type
                    queue = deque([(r, c, 0)])
                    self.box_heat_map[box_id][r][c] = 0
                    while queue:
                        curr_r, curr_c, dist = queue.popleft()
                        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                            nr, nc = curr_r + dr, curr_c + dc
                            if 0 <= nr < rows and 0 <= nc < cols and not State.walls[nr][nc]:
                                if self.box_heat_map[box_id][nr][nc] > dist + 1:
                                    self.box_heat_map[box_id][nr][nc] = dist + 1
                                    queue.append((nr, nc, dist + 1))

    def h(self, state: State) -> int:
        total_distance = 0
        w_box_to_goal = 5
        w_agent_to_box = 1

        for r in range(len(state.boxes)):
            for c in range(len(state.boxes[r])):
                box = state.boxes[r][c]
                if "A" <= box <= "Z":
                    box_id = ord(box) - ord("A")
                    if self.box_in_level[box_id]:
                        dist_to_goal = self.box_heat_map[box_id][r][c]
                        if dist_to_goal < self.inf:
                            total_distance += dist_to_goal * w_box_to_goal
                            
                            # Add distance to the nearest agent of the same color
                            min_agent_dist = self.inf
                            for a in range(len(state.agent_rows)):
                                if State.agent_colors[a] == State.box_colors[box_id]:
                                    d = abs(state.agent_rows[a] - r) + abs(state.agent_cols[a] - c)
                                    if d < min_agent_dist:
                                        min_agent_dist = d
                            if min_agent_dist < self.inf:
                                total_distance += min_agent_dist * w_agent_to_box
        return total_distance

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...

class HeuristicAStar(Heuristic):
    def f(self, state: State) -> int:
        return state.g + self.h(state)
    def __repr__(self) -> str:
        return "A* evaluation"

class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w
    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)
    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"

class HeuristicGreedy(Heuristic):
    def f(self, state: State) -> int:
        return self.h(state)
    def __repr__(self) -> str:
        return "greedy evaluation"