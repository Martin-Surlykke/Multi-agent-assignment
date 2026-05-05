import heapq
from abc import ABC, abstractmethod
from collections import deque

from searchclient.heuristic import Heuristic
from searchclient.state import State


class Frontier(ABC):
    @abstractmethod
    def add(self, state: State) -> None: ...

    @abstractmethod
    def pop(self) -> State: ...

    @abstractmethod
    def is_empty(self) -> bool: ...

    @abstractmethod
    def size(self) -> int: ...

    @abstractmethod
    def contains(self, state: State) -> bool: ...

    @abstractmethod
    def get_name(self) -> str: ...


class FrontierBFS(Frontier):
    def __init__(self) -> None:
        super().__init__()
        self.queue: deque[State] = deque()
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        self.queue.append(state)
        self.set.add(state)

    def pop(self) -> State:
        state = self.queue.popleft()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.queue) == 0

    def size(self) -> int:
        return len(self.queue)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "breadth-first search"


class FrontierDFS(Frontier):
    def __init__(self) -> None:
        super().__init__()
        self.stack: list[State] = []
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        # LIFO operation mimicking ArrayDeque.addFirst
        self.stack.append(state)
        self.set.add(state)

    def pop(self) -> State:
        # LIFO operation mimicking ArrayDeque.pollFirst
        state = self.stack.pop()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.stack) == 0

    def size(self) -> int:
        return len(self.stack)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "depth-first search"


class FrontierBestFirst(Frontier):
    def __init__(self, heuristic: Heuristic) -> None:
        super().__init__()
        self.heuristic = heuristic
        self.priority_queue = []  
        self.entry_finder = {}   
        self.counter = 0      

    def add(self, state: State) -> None:

        f_score = self.heuristic.f(state)

        entry = [f_score, self.counter, state]
        self.counter += 1
        heapq.heappush(self.priority_queue, entry)
        self.entry_finder[state] = entry

    def pop(self) -> State:
        while self.priority_queue:
            f, count, state = heapq.heappop(self.priority_queue)
            if state in self.entry_finder:
                del self.entry_finder[state]
                return state
        raise KeyError('Pop from empty frontier')

    def is_empty(self) -> bool:
        return len(self.entry_finder) == 0

    def size(self) -> int:
        return len(self.entry_finder)

    def contains(self, state: State) -> bool:
        return state in self.entry_finder

    def get_name(self) -> str:
        return f"best-first search using {self.heuristic}"