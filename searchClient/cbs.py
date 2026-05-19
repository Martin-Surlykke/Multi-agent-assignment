import heapq
import itertools
import sys
from collections import deque
from searchclient.action import Action, ActionType
from searchclient.state import State

class CBSNode: 
    def __init__ (self, constraints, solution, cost): 
        self.constraints = constraints
        self.solution = solution
        self.cost = cost 
    
    def __lt__ (self, other): 
        return self.cost < other.cost 


# Global 4D dictionary tracking precomputed heatmaps for every unique goal tile
GOAL_HEAT_MAPS = {}

def precompute_slot_heatmaps():
    """Runs a single-source BFS from every unique goal cell on the map."""
    global GOAL_HEAT_MAPS
    GOAL_HEAT_MAPS.clear()
    
    rows = len(State.walls)
    cols = len(State.walls[0]) if rows > 0 else 0

    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            goal_char = State.goals[r][c]
            if goal_char != "":
                dist_matrix = [[10000000 for _ in range(cols)] for _ in range(rows)]
                queue = deque([(r, c, 0)])
                dist_matrix[r][c] = 0
                
                while queue:
                    curr_r, curr_c, d = queue.popleft()
                    for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
                        nr, nc = curr_r + dr, curr_c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            if not State.walls[nr][nc] and dist_matrix[nr][nc] > d + 1:
                                dist_matrix[nr][nc] = d + 1
                                queue.append((nr, nc, d + 1))
                                
                GOAL_HEAT_MAPS[(r, c)] = dist_matrix

def get_occupancy_end_time(path: list[State], entity_type: str, a_id: int, pos: tuple, start_t: int) -> int | float:
    curr_t = start_t
    while True:
        state = path[curr_t] if curr_t < len(path) else path[-1]
        if entity_type == "agent":
            if (state.agent_rows[a_id], state.agent_cols[a_id]) != pos:
                return curr_t - 1
        elif entity_type == "box":
            b_char = state.boxes[pos[0]][pos[1]]
            if b_char == "":
                return curr_t - 1
            b_idx = ord(b_char) - ord("A")
            if State.box_colors[b_idx] != State.agent_colors[a_id]:
                return curr_t - 1
        
        curr_t += 1
        if curr_t > start_t + 50 or curr_t >= len(path) + 20:
            return float('inf')

def find_first_conflict(solution: list[list[State]]) -> tuple | None: 
    num_agents = len(solution) 
    max_len = max(len(path) for path in solution) 

    agent_positions = [[] for _ in range(num_agents)]
    box_positions_timeline = [[] for _ in range(num_agents)]

    for a in range(num_agents):
        path = solution[a]
        agent_color = State.agent_colors[a]
        for t in range(max_len):
            state = path[t] if t < len(path) else path[-1]
            agent_positions[a].append((state.agent_rows[a], state.agent_cols[a]))
            
            # CRITICAL FIX: Only track boxes that belong to this agent's color
            boxes_dict = {
                (r, c): val
                for r, row in enumerate(state.boxes)
                for c, val in enumerate(row)
                if val != "" and State.box_colors[ord(val) - ord('A')] == agent_color
            }
            box_positions_timeline[a].append(boxes_dict)

    for t in range(max_len): 
        for a1 in range(num_agents): 
            pos1 = agent_positions[a1][t]
            boxes1 = box_positions_timeline[a1][t]

            for a2 in range(num_agents): 
                if a1 == a2:
                    continue
                
                pos2 = agent_positions[a2][t]
                boxes2 = box_positions_timeline[a2][t]

                if a1 < a2 and pos1 == pos2: 
                    return ("vertex", a1, a2, pos1, t) 
                
                if a1 < a2:
                    for b_pos, b_char1 in boxes1.items():
                        if b_pos in boxes2 and boxes2[b_pos] != b_char1:
                            return ("box_vertex", a1, a2, b_pos, t)
                
                if pos1 in boxes2:
                    return ("agent_box", a1, a2, pos1, t)

        if t > 0:
            for a1 in range(num_agents):
                pos1 = agent_positions[a1][t]
                prev_pos1 = agent_positions[a1][t-1]
                boxes1 = box_positions_timeline[a1][t]
                prev_boxes1 = box_positions_timeline[a1][t-1]

                a1_moved_agent = pos1 if pos1 != prev_pos1 else None
                a1_new_boxes = set(boxes1.keys()) - set(prev_boxes1.keys())

                for a2 in range(num_agents):
                    if a1 == a2: 
                        continue
                    
                    pos2 = agent_positions[a2][t]
                    prev_pos2 = agent_positions[a2][t-1]
                    prev_boxes2 = box_positions_timeline[a2][t-1]

                    if a1 < a2 and pos1 == prev_pos2 and pos2 == prev_pos1:
                        return ("edge", a1, a2, ((prev_pos1, pos1), (prev_pos2, pos2)), t)

                    if a1_moved_agent and a1_moved_agent == prev_pos2:
                        return ("follow_agent_agent", a1, a2, a1_moved_agent, t)
                    
                    if a1_moved_agent and a1_moved_agent in prev_boxes2:
                        return ("follow_agent_box", a1, a2, a1_moved_agent, t)

                    for b_pos in a1_new_boxes:
                        if b_pos == prev_pos2:
                            return ("follow_box_agent", a1, a2, b_pos, t)
                        if b_pos in prev_boxes2:
                            return ("follow_box_box", a1, a2, b_pos, t)
    return None


def get_occupancy_end_time(path: list[State], entity_type: str, a_id: int, pos: tuple, start_t: int) -> int | float:
    curr_t = start_t
    while True:
        state = path[curr_t] if curr_t < len(path) else path[-1]
        if entity_type == "agent":
            if (state.agent_rows[a_id], state.agent_cols[a_id]) != pos:
                return curr_t - 1
        elif entity_type == "box":
            if state.boxes[pos[0]][pos[1]] == "":
                return curr_t - 1
        
        curr_t += 1
        if curr_t > start_t + 50 or curr_t >= len(path) + 20:
            return float('inf')


def cbs_search(initial_state: State) -> list[list[Action]] | None: 
    precompute_slot_heatmaps()

    num_agents = len(initial_state.agent_rows)
    print(f"[CBS] Initializing single-agent paths for {num_agents} agents...", file=sys.stderr, flush=True)

    root_constraints = []
    root_solution = [] 
    empty_congestion = {}

    for a in range(num_agents): 
        path = low_level_search(a, initial_state, root_constraints, empty_congestion) 
        if path is None: 
            print(f"[CBS] Initial search failed: Agent {a} cannot find a path to its goals.", file=sys.stderr, flush=True)
            return None
        root_solution.append(path) 
    root_cost = sum(len(path) - 1 for path in root_solution)

    root_node = CBSNode(root_constraints, root_solution, root_cost)
    open_list = [] 
    heapq.heappush(open_list, root_node)

    hl_expanded = 0

    while open_list: 
        P = heapq.heappop(open_list)
        hl_expanded += 1

        congestion_map = setup_congestion_map(P.solution)
        conflict = find_first_conflict(P.solution)

        if hl_expanded % 10 == 0 or not conflict:
            print(f"[CBS High-Level] Expanded Nodes: {hl_expanded} | Open List Size: {len(open_list)} | Current Node Cost: {P.cost}", file=sys.stderr, flush=True)

        if not conflict:
            print(f"[CBS Success] Conflict-free joint plan found after expanding {hl_expanded} high-level nodes.", file=sys.stderr, flush=True)
            return reconstruct_joint_plan(P.solution)
        
        conflict_type, agent1, agent2, details, t = conflict 
        print(f"  -> Found Conflict: type='{conflict_type}', agents=({agent1}, {agent2}), details={details}, timestep={t}", file=sys.stderr, flush=True)

        for agent_id in [agent1, agent2]: 
            new_constraints = []

            if conflict_type == "vertex":
                opp_agent = agent2 if agent_id == agent1 else agent1
                end_t = get_occupancy_end_time(P.solution[opp_agent], "agent", opp_agent, details, t)
                if end_t == float('inf'):
                    new_constraints = [(agent_id, "agent_vertex_permanent", details, t)]
                else:
                    new_constraints = [(agent_id, "agent_vertex", details, step_t) for step_t in range(t, int(end_t) + 1)]
            
            elif conflict_type == "edge": 
                agent_edge = details[0] if agent_id == agent1 else details[1]
                new_constraints = [(agent_id, "agent_edge", agent_edge, t)]
            
            elif conflict_type == "box_vertex":
                opp_agent = agent2 if agent_id == agent1 else agent1
                end_t = get_occupancy_end_time(P.solution[opp_agent], "box", opp_agent, details, t)
                if end_t == float('inf'):
                    new_constraints = [(agent_id, "box_vertex_permanent", details, t)]
                else:
                    new_constraints = [(agent_id, "box_vertex", details, step_t) for step_t in range(t, int(end_t) + 1)]
                
            elif conflict_type == "agent_box":
                if agent_id == agent1:
                    end_t = get_occupancy_end_time(P.solution[agent2], "box", agent2, details, t)
                    if end_t == float('inf'):
                        new_constraints = [(agent_id, "agent_vertex_permanent", details, t)]
                    else:
                        new_constraints = [(agent_id, "agent_vertex", details, step_t) for step_t in range(t, int(end_t) + 1)]
                else:
                    end_t = get_occupancy_end_time(P.solution[agent1], "agent", agent1, details, t)
                    if end_t == float('inf'):
                        new_constraints = [(agent_id, "box_vertex_permanent", details, t)]
                    else:
                        new_constraints = [(agent_id, "box_vertex", details, step_t) for step_t in range(t, int(end_t) + 1)]

            elif conflict_type.startswith("follow"):
                if agent_id == agent1:
                    c_type = "agent_vertex" if conflict_type in ["follow_agent_agent", "follow_agent_box"] else "box_vertex"
                    new_constraints = [(agent_id, c_type, details, t)]
                else:
                    c_type = "agent_vertex" if conflict_type in ["follow_agent_agent", "follow_box_agent"] else "box_vertex"
                    new_constraints = [(agent_id, c_type, details, t - 1)]        
            
            next_constraints = P.constraints + new_constraints
            next_solution = list(P.solution)
            
            print(f"     * Replanning Agent {agent_id} with {len(new_constraints)} constraint(s) up to t={t if not new_constraints or 'permanent' in new_constraints[0][1] else new_constraints[-1][3]}", file=sys.stderr, flush=True)
            new_path = low_level_search(agent_id, initial_state, next_constraints, congestion_map)

            if new_path is not None: 
                next_solution[agent_id] = new_path
                next_cost = sum(len(path) - 1 for path in next_solution)

                A = CBSNode(next_constraints, next_solution, next_cost)
                heapq.heappush(open_list, A)
            else:
                print(f"     * Replanning failed for Agent {agent_id}. Branch pruned.", file=sys.stderr, flush=True)
                
    print("[CBS Failure] Open list empty. No joint plan exists within constraints.", file=sys.stderr, flush=True)
    return None


def reconstruct_joint_plan(solution: list[list[State]]) -> list[list[Action]]: 
    num_agents = len(solution)
    max_len = max(len(path) for path in solution) 
    joint_plan = [] 

    for t in range(max_len - 1): 
        step_actions = [] 
        for a in range(num_agents): 
            path = solution[a]
            if t < len(path) - 1: 
                step_actions.append(path[t+1].individual_action)
            else: 
                step_actions.append(Action.NoOp)
        joint_plan.append(step_actions)
    return joint_plan


def low_level_search(agent_id: int, initial_state: State, constraints: list, congestion_map: dict) -> list[State] | None: 
    num_agents = len(initial_state.agent_rows)
    agent_color = State.agent_colors[agent_id]

    # Precompute Indexing
    indexed_constraints = {}
    max_constraint_t = 0
    for con_agent, con_type, details, con_t in constraints:
        if con_agent == agent_id:
            if con_t not in indexed_constraints:
                indexed_constraints[con_t] = []
            indexed_constraints[con_t].append((con_type, details))
            if con_t > max_constraint_t:
                max_constraint_t = con_t

    # Precompute Goals
    agent_goal_pos = None
    agent_box_goals = {}
    for r in range(len(State.goals)):
        for c in range(len(State.goals[r])):
            goal = State.goals[r][c]
            if goal == str(agent_id):
                agent_goal_pos = (r, c)
            elif "A" <= goal <= "Z":
                box_id = ord(goal) - ord("A")
                if State.box_colors[box_id] == agent_color:
                    agent_box_goals[(r, c)] = goal

    # Initial State Setup
    start_ar = initial_state.agent_rows[agent_id]
    start_ac = initial_state.agent_cols[agent_id] 
    
    initial_boxes_dict = {}
    for r in range(len(initial_state.boxes)):
        for c in range(len(initial_state.boxes[r])): 
            box = initial_state.boxes[r][c]
            if box != "": 
                box_id = ord(box) - ord("A")
                if State.box_colors[box_id] == agent_color: 
                    initial_boxes_dict[(r, c)] = box

    # Use immutable frozenset to track box state
    start_boxes_fs = frozenset(initial_boxes_dict.items())

    if violates_constraints_indexed(agent_id, start_ar, start_ac, 0, -1, -1, start_boxes_fs, indexed_constraints):
        return None

    start_h = compute_h(agent_id, start_ar, start_ac, initial_boxes_dict, agent_goal_pos, agent_box_goals)
    
    open_list = []
    counter = itertools.count()
    
    start_key = ((start_ar, start_ac, start_boxes_fs), 0 if max_constraint_t >= 0 else max_constraint_t + 1)
    
    heapq.heappush(open_list, (start_h, start_h, 0, next(counter), 0, start_ar, start_ac, start_boxes_fs, start_key))

    g_score = {start_key: 0}
    came_from = {}

    while open_list:
        _, child_h, _, _, current_g, ar, ac, current_boxes_fs, current_key = heapq.heappop(open_list)
        
        if current_g > g_score.get(current_key, float('inf')):
            continue

        if child_h == 0 and current_g >= max_constraint_t:
            actions = []
            curr = current_key
            while curr in came_from:
                curr, act = came_from[curr]
                actions.append(act)
            actions.reverse()
            
            path = [initial_state]
            curr_state = initial_state
            for act in actions:
                joint_action = [Action.NoOp] * num_agents
                joint_action[agent_id] = act
                nxt = curr_state.result(joint_action)
                nxt.individual_action = act
                path.append(nxt)
                curr_state = nxt 
            return path

        if current_g >= 1000:
            continue

        # Create dictionary representation once per expanded node
        curr_boxes_dict = dict(current_boxes_fs)

        for action in Action:
            next_ar = ar + action.agent_row_delta
            next_ac = ac + action.agent_col_delta
            
            # Fast wall check
            if action.type != ActionType.NoOp and State.walls[next_ar][next_ac]:
                continue

            next_boxes_fs = current_boxes_fs

            if action.type == ActionType.Move:
                if (next_ar, next_ac) in curr_boxes_dict: 
                    continue
            elif action.type == ActionType.Push:
                if (next_ar, next_ac) not in curr_boxes_dict: 
                    continue
                box_new_r = next_ar + action.box_row_delta
                box_new_c = next_ac + action.box_col_delta
                if State.walls[box_new_r][box_new_c] or (box_new_r, box_new_c) in curr_boxes_dict: 
                    continue
                
                # O(1) set operations instead of O(N) dict copying
                b_char = curr_boxes_dict[(next_ar, next_ac)]
                next_boxes_fs = (current_boxes_fs - {((next_ar, next_ac), b_char)}) | {((box_new_r, box_new_c), b_char)}
                
            elif action.type == ActionType.Pull:
                if (next_ar, next_ac) in curr_boxes_dict: 
                    continue
                box_r = ar - action.box_row_delta
                box_c = ac - action.box_col_delta
                if (box_r, box_c) not in curr_boxes_dict: 
                    continue
                
                b_char = curr_boxes_dict[(box_r, box_c)]
                next_boxes_fs = (current_boxes_fs - {((box_r, box_c), b_char)}) | {((ar, ac), b_char)}

            next_g = current_g + 1

            if violates_constraints_indexed(agent_id, next_ar, next_ac, next_g, ar, ac, next_boxes_fs, indexed_constraints):
                continue

            next_key = ((next_ar, next_ac, next_boxes_fs), next_g if next_g <= max_constraint_t else max_constraint_t + 1)

            if next_g < g_score.get(next_key, float('inf')):
                g_score[next_key] = next_g
                came_from[next_key] = (current_key, action)
                
                next_boxes_dict = dict(next_boxes_fs)
                next_h = compute_h(agent_id, next_ar, next_ac, next_boxes_dict, agent_goal_pos, agent_box_goals)
                c_pen = congestion_map.get((next_ar, next_ac), 0)
                f_value = next_g + next_h
                
                heapq.heappush(open_list, (f_value, next_h, c_pen, next(counter), next_g, next_ar, next_ac, next_boxes_fs, next_key))
                
    return None


def violates_constraints_indexed(agent_id: int, state_r: int, state_c: int, state_t: int, parent_r: int, parent_c: int, next_boxes_fs: frozenset, indexed_constraints: dict) -> bool:
    for c_type, details in indexed_constraints.get(state_t, []):
        if c_type == "agent_vertex" and (state_r, state_c) == details:
            return True
        elif c_type == "box_vertex":
            br, bc = details
            if any(pos == (br, bc) for pos, _ in next_boxes_fs):
                return True
        elif c_type == "agent_edge" and parent_r != -1:
            if ((parent_r, parent_c), (state_r, state_c)) == details:
                return True

    for con_t, constraints in indexed_constraints.items():
        if state_t >= con_t:
            for c_type, details in constraints:
                if c_type == "agent_vertex_permanent" and (state_r, state_c) == details:
                    return True
                elif c_type == "box_vertex_permanent":
                    br, bc = details
                    if any(pos == (br, bc) for pos, _ in next_boxes_fs):
                        return True
    return False


def compute_h(agent_id: int, r: int, c: int, box_map: dict, agent_goal_pos: tuple, agent_box_goals: dict) -> int:
    total_distance = 0
    if agent_goal_pos:
        total_distance += GOAL_HEAT_MAPS[agent_goal_pos][r][c]

    has_boxes = False
    for (gr, gc), goal_char in agent_box_goals.items():
        if box_map.get((gr, gc)) == goal_char:
            continue
            
        min_box_dist = 10000000
        for (br, bc), b_char in box_map.items():
            if b_char == goal_char:
                d = GOAL_HEAT_MAPS[(gr, gc)][br][bc]
                if d < min_box_dist:
                    min_box_dist = d
                    
        if min_box_dist < 10000000:
            total_distance += min_box_dist
            has_boxes = True

    if has_boxes:
        min_agent_to_box = 10000000
        for br, bc in box_map.keys():
            dist_to_agent = abs(r - br) + abs(c - bc) - 1
            if dist_to_agent < min_agent_to_box:
                min_agent_to_box = dist_to_agent
        if min_agent_to_box > 0 and min_agent_to_box < 10000000:
            total_distance += min_agent_to_box

    return total_distance


def violates_constraints(agent_id: int, state: State, parent_state: State | None, constraints: list) -> bool: 
    t = state.g
    r = state.agent_rows[agent_id]
    c = state.agent_cols[agent_id]

    for constraint_agent, constraint_type, details, con_t in constraints: 
        if constraint_agent != agent_id: 
            continue

        if "permanent" in constraint_type:
            if t < con_t:
                continue
        else:
            if t != con_t:
                continue

        if constraint_type in ["agent_vertex", "agent_vertex_permanent"]: 
            if (r, c) == details: 
                return True
            
        elif constraint_type in ["box_vertex", "box_vertex_permanent"]: 
            br, bc = details
            if 0 <= br < len(state.boxes) and 0 <= bc < len(state.boxes[br]):
                if state.boxes[br][bc] != "": 
                    return True

        elif constraint_type == "agent_edge": 
            if parent_state is not None: 
                pr = parent_state.agent_rows[agent_id]
                pc = parent_state.agent_cols[agent_id]
                if ((pr, pc), (r, c)) == details: 
                    return True
    return False 

def setup_congestion_map(solution: list[list[State]]) -> dict[tuple, int]: 
    congestion = {}
    for path in solution: 
        for state in path: 
            for agent in range(len(state.agent_rows)): 
                r = state.agent_rows[agent]
                c = state.agent_cols[agent]
                if r != -1 and c != -1: 
                    pos = (r, c)
                    congestion[pos] = congestion.get(pos, 0) + 1
    return congestion

