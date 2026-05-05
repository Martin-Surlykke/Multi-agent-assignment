import random
from typing import ClassVar

from searchclient.action import Action, ActionType
from searchclient.color import Color


class State:
    _RNG = random.Random(1)

    agent_colors: ClassVar[list[Color | None]]
    walls: ClassVar[list[list[bool]]]
    box_colors: ClassVar[list[Color | None]]
    goals: ClassVar[list[list[str]]]

    def __init__(self, agent_rows: list[int], agent_cols: list[int], boxes: list[list[str]]) -> None:
        self.agent_rows = agent_rows
        self.agent_cols = agent_cols
        self.boxes = boxes
        self.parent: State | None = None
        self.joint_action: list[Action] | None = None
        self.g = 0
        self._hash: int | None = None

    def result(self, joint_action: list[Action]) -> "State":
        copy_agent_rows = self.agent_rows[:]
        copy_agent_cols = self.agent_cols[:]
        copy_boxes = [row[:] for row in self.boxes]

        for agent, action in enumerate(joint_action):
            old_agent_row = self.agent_rows[agent]
            old_agent_col = self.agent_cols[agent]

            if action.type is ActionType.NoOp:
                pass

            elif action.type is ActionType.Move:
                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

            elif action.type is ActionType.Push:
                box_old_row = old_agent_row + action.agent_row_delta
                box_old_col = old_agent_col + action.agent_col_delta

                box_new_row = box_old_row + action.box_row_delta
                box_new_col = box_old_col + action.box_col_delta

                push_box = copy_boxes[box_old_row][box_old_col]
                copy_boxes[box_old_row][box_old_col] = ""
                copy_boxes[box_new_row][box_new_col] = push_box

                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

            elif action.type is ActionType.Pull:
                p_box_old_row = old_agent_row - action.box_row_delta
                p_box_old_col = old_agent_col - action.box_col_delta

                pull_box = copy_boxes[p_box_old_row][p_box_old_col]
                copy_boxes[p_box_old_row][p_box_old_col] = ""
                copy_boxes[old_agent_row][old_agent_col] = pull_box

                copy_agent_rows[agent] += action.agent_row_delta
                copy_agent_cols[agent] += action.agent_col_delta

        copy_state = State(copy_agent_rows, copy_agent_cols, copy_boxes)
        copy_state.parent = self
        copy_state.joint_action = joint_action.copy()
        copy_state.g = self.g + 1

        return copy_state

    def is_goal_state(self) -> bool:
        for row in range(1, len(State.goals) - 1):
            for col in range(1, len(State.goals[row]) - 1):
                goal = State.goals[row][col]

                if "A" <= goal <= "Z" and self.boxes[row][col] != goal:
                    return False
                elif "0" <= goal <= "9" and not (
                    self.agent_rows[ord(goal) - ord("0")] == row and self.agent_cols[ord(goal) - ord("0")] == col
                ):
                    return False
        return True

    def get_expanded_states(self) -> list["State"]:
        num_agents = len(self.agent_rows)

        applicable_actions = [
            [action for action in Action if self.is_applicable(agent, action)] for agent in range(num_agents)
        ]

        joint_action = [Action.NoOp for _ in range(num_agents)]
        actions_permutation = [0 for _ in range(num_agents)]
        expanded_states = []
        
        while True:
            for agent in range(num_agents):
                joint_action[agent] = applicable_actions[agent][actions_permutation[agent]]

            if not self.is_conflicting(joint_action):
                expanded_states.append(self.result(joint_action))

            done = False
            for agent in range(num_agents):
                if actions_permutation[agent] < len(applicable_actions[agent]) - 1:
                    actions_permutation[agent] += 1
                    break
                else:  
                    actions_permutation[agent] = 0
                    if agent == num_agents - 1:
                        done = True

            if done:
                break

        State._RNG.shuffle(expanded_states)
        return expanded_states

    def is_applicable(self, agent: int, action: Action) -> bool:
        agent_row = self.agent_rows[agent]
        agent_col = self.agent_cols[agent]
        agent_color = State.agent_colors[agent]

        if action.type is ActionType.NoOp:
            return True

        elif action.type is ActionType.Move:
            destination_row = agent_row + action.agent_row_delta
            destination_col = agent_col + action.agent_col_delta
            return self.is_free(destination_row, destination_col)

        elif action.type is ActionType.Push:
            box_row = agent_row + action.agent_row_delta
            box_col = agent_col + action.agent_col_delta

            if box_row < 0 or box_row >= len(self.boxes) or box_col < 0 or box_col >= len(self.boxes[0]):
                return False
                
            box = self.boxes[box_row][box_col]
            if box == "" or not ("A" <= box <= "Z"):
                return False

            if State.box_colors[ord(box) - ord('A')] != agent_color:
                return False

            destination_row = box_row + action.box_row_delta
            destination_col = box_col + action.box_col_delta
            return self.is_free(destination_row, destination_col)

        elif action.type is ActionType.Pull:
            pull_agent_dest_row = agent_row + action.agent_row_delta
            pull_agent_dest_col = agent_col + action.agent_col_delta

            if not self.is_free(pull_agent_dest_row, pull_agent_dest_col):
                return False

            pull_box_row = agent_row - action.box_row_delta
            pull_box_col = agent_col - action.box_col_delta

            if pull_box_row < 0 or pull_box_row >= len(self.boxes) or pull_box_col < 0 or pull_box_col >= len(self.boxes[0]):
                return False

            pull_box_char = self.boxes[pull_box_row][pull_box_col]
            if pull_box_char == "" or not ("A" <= pull_box_char <= "Z"):
                return False

            return State.box_colors[ord(pull_box_char) - ord('A')] == agent_color

        return False

    def is_conflicting(self, joint_action: list[Action]) -> bool:
        num_agents = len(self.agent_rows)
        agent_dest_rows = [-1 for _ in range(num_agents)]
        agent_dest_cols = [-1 for _ in range(num_agents)]
        box_dest_rows = [-1 for _ in range(num_agents)]
        box_dest_cols = [-1 for _ in range(num_agents)]

        for agent in range(num_agents):
            action = joint_action[agent]
            agent_dest_rows[agent] = self.agent_rows[agent] + action.agent_row_delta
            agent_dest_cols[agent] = self.agent_cols[agent] + action.agent_col_delta

            if action.type is ActionType.Push:
                box_dest_rows[agent] = self.agent_rows[agent] + action.agent_row_delta + action.box_row_delta
                box_dest_cols[agent] = self.agent_cols[agent] + action.agent_col_delta + action.box_col_delta
            elif action.type is ActionType.Pull:
                box_dest_rows[agent] = self.agent_rows[agent]
                box_dest_cols[agent] = self.agent_cols[agent]

        for a1 in range(num_agents):
            for a2 in range(a1 + 1, num_agents):
                if agent_dest_rows[a1] == agent_dest_rows[a2] and agent_dest_cols[a1] == agent_dest_cols[a2]:
                    return True

                if box_dest_rows[a1] != -1 and box_dest_rows[a1] == box_dest_rows[a2] and box_dest_cols[a1] == box_dest_cols[a2]:
                    return True

                if box_dest_rows[a2] != -1 and agent_dest_rows[a1] == box_dest_rows[a2] and agent_dest_cols[a1] == box_dest_cols[a2]:
                    return True
                if box_dest_rows[a1] != -1 and agent_dest_rows[a2] == box_dest_rows[a1] and agent_dest_cols[a2] == box_dest_cols[a1]:
                    return True

                if (agent_dest_rows[a1] == self.agent_rows[a2] and agent_dest_cols[a1] == self.agent_cols[a2] and
                        agent_dest_rows[a2] == self.agent_rows[a1] and agent_dest_cols[a2] == self.agent_cols[a1]):
                    return True

        return False

    def is_free(self, row: int, col: int) -> bool:
        return not State.walls[row][col] and self.boxes[row][col] == "" and self.agent_at(row, col) is None

    def agent_at(self, row: int, col: int) -> str | None:
        for agent in range(len(self.agent_rows)):
            if self.agent_rows[agent] == row and self.agent_cols[agent] == col:
                return chr(agent + ord("0"))
        return None

    def extract_plan(self) -> list[list[Action]]:
        plan = []
        state: State | None = self
        while state is not None and state.joint_action is not None:
            plan.append(state.joint_action)
            state = state.parent
        plan.reverse()
        return plan

    def __hash__(self) -> int:
        if self._hash is None:
            prime = 31
            h = 1
            h = h * prime + hash(tuple(self.agent_rows))
            h = h * prime + hash(tuple(self.agent_cols))
            h = h * prime + hash(tuple(State.agent_colors))
            h = h * prime + hash(tuple(tuple(row) for row in self.boxes))
            h = h * prime + hash(tuple(State.box_colors))
            h = h * prime + hash(tuple(tuple(row) for row in State.goals))
            h = h * prime + hash(tuple(tuple(row) for row in State.walls))
            self._hash = h
        return self._hash

    def __eq__(self, other: object) -> bool:
        if self is other:
            return True
        if not isinstance(other, State):
            return False
        if self.agent_rows != other.agent_rows:
            return False
        if self.agent_cols != other.agent_cols:
            return False
        if State.agent_colors != other.agent_colors:
            return False
        if State.walls != other.walls:
            return False
        if self.boxes != other.boxes:
            return False
        if State.box_colors != other.box_colors:
            return False
        return State.goals == other.goals

    def __repr__(self) -> str:
        lines = []
        for row in range(len(self.boxes)):
            line = []
            for col in range(len(self.boxes[row])):
                if self.boxes[row][col]:
                    line.append(self.boxes[row][col])
                elif State.walls[row][col]:
                    line.append("+")
                elif (agent := self.agent_at(row, col)) is not None:
                    line.append(agent)
                else:
                    line.append(" ")
            lines.append("".join(line))
        return "\n".join(lines)