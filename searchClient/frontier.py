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
        raise NotImplementedError

    def add(self, state: State) -> None:
        raise NotImplementedError

    def pop(self) -> State:
        raise NotImplementedError

    def is_empty(self) -> bool:
        raise NotImplementedError

    def size(self) -> int:
        raise NotImplementedError

    def contains(self, state: State) -> bool:
        raise NotImplementedError

    def get_name(self) -> str:
        return "depth-first search"


class FrontierBestFirst(Frontier):
    def __init__(self, heuristic: Heuristic) -> None:
        super().__init__()
        self.heuristic = heuristic
        raise NotImplementedError

    def add(self, state: State) -> None:
        raise NotImplementedError

    def pop(self) -> State:
        raise NotImplementedError

    def is_empty(self) -> bool:
        raise NotImplementedError

    def size(self) -> int:
        raise NotImplementedError

    def contains(self, state: State) -> bool:
        raise NotImplementedError

    def get_name(self) -> str:
        return f"best-first search using {self.heuristic}"
