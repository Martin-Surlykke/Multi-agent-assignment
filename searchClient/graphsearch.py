import sys
import time
from searchclient import memory
from searchclient.action import Action
from searchclient.frontier import Frontier
from searchclient.state import State

start_time = time.perf_counter()

def search(initial_state: State, frontier: Frontier) -> list[list[Action]] | None:
    expanded_count = 0
    generated_count = 0
    
    # Initial state setup
    frontier.add(initial_state)
    generated_count += 1
    explored = set()

    while not frontier.is_empty():

        if expanded_count > 0 and expanded_count % 1000 == 0:
            print_stats(expanded_count, frontier.size(), generated_count)

        leaf_state = frontier.pop()


        if leaf_state.is_goal_state():
            print_stats(expanded_count, frontier.size(), generated_count)
            return leaf_state.extract_plan()

        explored.add(leaf_state)
        expanded_count += 1

        for child in leaf_state.get_expanded_states():
            generated_count += 1
            if child not in explored and not frontier.contains(child):
                frontier.add(child)

    return None

def print_stats(expanded, frontier_size, generated):
    elapsed_time = time.perf_counter() - start_time
    print(f"#Expanded: {expanded:8,}, #Frontier: {frontier_size:8,}, #Generated: {generated:8,}, Time: {elapsed_time:3.3f} s", 
          file=sys.stderr, flush=True)
    print(f"[Used: {memory.get_usage():.2f} MB, Max: {memory.max_usage:.2f} MB]", 
          file=sys.stderr, flush=True)