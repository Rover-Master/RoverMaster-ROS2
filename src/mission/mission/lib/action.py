# ==============================================================================
# Author: Yuxuan Zhang (robotics@z-yx.cc)
# License: MIT
# ==============================================================================
from typing import Generic, TypeVar, Generator, Union
from threading import Thread
from functools import wraps
from sys import stderr

from .stack import Stack
from .queue import Queue

T = TypeVar("T")


def purify(callback):
    def wrapper(arg):
        if arg is not None and callback is not None:
            callback(arg)

    return wrapper


class Action(Generic[T]):

    class Hub(Generic[T]):

        def __init__(self, *_, **__):
            super().__init__(*_, **__)
            self.__action_stack__ = Stack["Action[T]"]()

        @staticmethod
        def add(dst: "Action.Hub[T]", action: "Action[T]"):
            dst.__action_stack__.push(action)

        def wait_action(self):
            stack = self.__action_stack__
            stack.pop_until(lambda a: not a.complete)
            action = stack.top()
            if action is not None:
                action()
                return True
            return False

    class Thread(Hub[T]):
        """
        The queue containing all yield values from actions
        """

        __action_yield__: Queue[T] = Queue()

        def wait_action(self, callback: callable):
            pending = len(self.__action_stack__) > 0
            return self.__action_yield__(purify(callback)) or pending

        def __init__(self):
            @Queue.Loop()
            def execute():
                stack = self.__action_stack__
                while True:
                    action = stack.top()
                    if action is None:
                        continue
                    try:
                        ret = next(action.task)
                    except StopIteration as e:
                        stack.pop()
                        ret = e.value
                    self.__action_yield__.put(ret)

            self.__action_thread__ = Thread(target=execute, daemon=True)
            self.__action_thread__.start()

        def __del__(self):
            self.__action_yield__.close()
            self.__action_thread__.join()

    @staticmethod
    def action(fn=None):

        def factory(slf, *args, **kwargs):
            task = fn(slf, *args, **kwargs)
            assert isinstance(task, Generator)
            print(f"Action {fn.__name__} created", file=stderr)
            Action.Hub.add(slf, Action(fn.__name__, task))

        return wraps(fn)(factory)

    def __repr__(self):
        return f"<Action {self.name}>"

    def __init__(
        self,
        name: str,
        task: Generator[Union["Action[T]", None], None, Union["Action[T]", None]],
    ):
        self.name = name
        self.task = task

    complete: bool = False

    def __call__(self):
        assert not self.complete, f"Action {self.name} already completed"
        try:
            return next(self.task)
        except StopIteration:
            self.complete = True

    def __del__(self):
        if not self.complete:
            print(f"Action {self.name} dropped before complete", file=stderr)

