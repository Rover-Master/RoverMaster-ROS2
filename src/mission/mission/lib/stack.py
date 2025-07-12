# ==============================================================================
# Author: Yuxuan Zhang (robotics@z-yx.cc)
# License: MIT
# ==============================================================================
from typing import Generic, TypeVar, Callable
from threading import Lock

T = TypeVar("T")

class Stack(Generic[T]):
    """
    A thread-safe stack implementation
    """
    def __init__(self):
        self.__stack: list[T] = []
        self.__lock = Lock()

    def push(self, item: T):
        with self.__lock:
            self.__stack.append(item)

    def pop(self):
        with self.__lock:
            return self.__stack.pop()
    
    def pop_until(self, fn: Callable[[T], bool]):
        items = list[T]()
        with self.__lock:
            while len(self.__stack) and not fn(self.__stack[-1]):
                items.append(self.__stack.pop())
        return items

    def remove(self, item: T):
        with self.__lock:
            while item in self.__stack:
                self.__stack.remove(item)
    
    def top(self):
        with self.__lock:
            if not len(self.__stack):
                return None
            return self.__stack[-1]

    def __len__(self):
        with self.__lock:
            return len(self.__stack)

    def __iter__(self):
        """
        Pop all items from the stack and yield them in first-in-last-out order
        """
        with self.__lock:
            while len(self.__stack):
                yield self.__stack.pop()

