from abc import ABC, abstractmethod
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    def __init__(self, initial_state: State) -> None:
        rows = len(State.walls)
        cols = len(State.walls[0]) if rows > 0 else 0

        self.box_heat_map = [[[10000000 for _ in range(cols)] for _ in range(rows)] for _ in range(26)]
        self.box_in_level = [False for _ in range(26)]

        box_queues: list[deque[tuple[int, int, int]]] = [deque() for _ in range(26)]

        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                goal = State.goals[r][c]
                if "A" <= goal <= "Z":
                    box_id = ord(goal) - ord("A")
                    box_queues[box_id].append((r, c, 0))
                    self.box_heat_map[box_id][r][c] = 0
                    self.box_in_level[box_id] = True

        neighbours = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        for i in range(26):
            if self.box_in_level[i]:
                queue = box_queues[i]

                while queue:
                    row, col, dist = queue.popleft()
                    new_dist = dist + 1

                    for dr, dc in neighbours:
                        n_row = row + dr
                        n_col = col + dc

                        if (
                            0 <= n_row < rows
                            and 0 <= n_col < cols
                            and not State.walls[n_row][n_col]
                            and self.box_heat_map[i][n_row][n_col] > new_dist
                        ):
                            self.box_heat_map[i][n_row][n_col] = new_dist
                            queue.append((n_row, n_col, new_dist))

    def h(self, state: State) -> int:
        total_distance = 0
        rows = len(State.walls)
        cols = len(State.walls[0]) if rows > 0 else 0

        w_box_to_goal = 5
        w_agent_to_box = 1

        for r in range(rows):
            for c in range(cols):
                box = state.boxes[r][c]

                if "A" <= box <= "Z":
                    box_id = ord(box) - ord("A")

                    if self.box_in_level[box_id]:
                        dist_to_goal = self.box_heat_map[box_id][r][c]

                        if dist_to_goal < 10000000:
                            total_distance += dist_to_goal * w_box_to_goal

                            min_agent_dist = 10000000
                            for a in range(len(state.agent_rows)):
                                if State.agent_colors[a] == State.box_colors[box_id]:
                                    dist_to_agent = abs(state.agent_rows[a] - r) + abs(state.agent_cols[a] - c)
                                    if dist_to_agent < min_agent_dist:
                                        min_agent_dist = dist_to_agent

                            total_distance += min_agent_dist * w_agent_to_box

        return total_distance

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

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
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"