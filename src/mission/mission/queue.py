# ==============================================================================
# Author: Yuxuan Zhang (robotics@z-yx.cc)
# License: MIT
# ==============================================================================
from typing import TypeVar
from time import time
from queue import Queue as StdQueue, Empty, Full
from collections import deque
from threading import Lock


T = TypeVar("T")


class __Guard__:
    def __init__(self, lock: Lock, exception: Exception, checker: callable):
        self.lock = lock
        self.exception = exception
        self.checker = checker

    def __call__(self):
        if self.checker():
            raise self.exception()

    def __enter__(self):
        self.lock.__enter__()
        if self.checker():
            self.lock.__exit__(None, None, None)
            raise self.exception()
        return self

    def __exit__(self, *args, **kwargs):
        return self.lock.__exit__(*args, **kwargs)


class Queue(StdQueue[T]):

    queue: deque[T]

    def __init__(self, drop: bool = False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drop = drop
        closed = False

        def close():
            nonlocal closed
            with self.mutex:
                closed = True
                self.not_empty.notify_all()
                self.not_full.notify_all()
            return self

        self.close = close
        self.__readable__ = __Guard__(
            lock=self.mutex,
            exception=self.Closed,
            checker=lambda: closed and not self._qsize(),
        )
        self.__writable__ = __Guard__(
            lock=self.mutex, exception=self.Closed, checker=lambda: closed
        )

    def dump(self) -> deque[T]:
        with self.__readable__:
            assert isinstance(self.queue, deque), type(self.queue)
            queue, self.queue = self.queue, deque()
        return queue

    def put(self, item: T, block=True, timeout: float = None):
        with self.__writable__ as check_writable:
            if self.maxsize > 0:
                if not block:
                    if self._qsize() >= self.maxsize:
                        raise Full
                elif timeout is None:
                    while self._qsize() >= self.maxsize:
                        self.not_full.wait()
                        check_writable()
                elif timeout < 0:
                    raise ValueError("'timeout' must be a non-negative number")
                else:
                    endtime = time() + timeout
                    while self._qsize() >= self.maxsize:
                        remaining = endtime - time()
                        if remaining <= 0.0:
                            raise Full
                        self.not_full.wait(remaining)
                        check_writable()
            if self.drop and self._qsize():
                self.queue.clear()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

    def get(self, block=True, timeout: float = None):
        with self.__readable__ as check_readable:
            if not block:
                if not self._qsize():
                    raise Empty
            elif timeout is None:
                while not self._qsize():
                    self.not_empty.wait()
                    check_readable()
            elif timeout < 0:
                raise ValueError("'timeout' must be a non-negative number")
            else:
                while not self._qsize():
                    self.not_empty.wait()
                    check_readable()
            item = self._get()
            self.not_full.notify()
            return item

    def pop(self, index: int):
        with self.__readable__:
            return self.queue.pop(index)

    def clear(self):
        with self.__readable__:
            q: deque[T] = self.queue
            q.clear()
            self.not_full.notify_all()
        return self

    def __iter__(self):
        """
        Listen on the queue until it is closed.
        """
        with self.Loop():
            while True:
                yield self.get(block=True, timeout=None)

    def __call__(self, callback: callable):
        """
        Try to get an item from the queue.
        Call the callback function if success.
        Returns whether the callback is executed.
        """
        try:
            callback(self.get(block=False))
            return True
        except Empty:
            return False

    def __getitem__(self, item):
        with self.__readable__:
            return self.queue[item]

    def __len__(self):
        with self.__readable__:
            return len(self.queue)

    Empty = Empty

    class Closed(Exception):
        pass

    class Loop:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            return exc_type is Queue.Closed

        def __call__(self, fn):
            def decorator(*args, **kwargs):
                with self:
                    return fn(*args, **kwargs)

            return decorator

