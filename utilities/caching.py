import functools
from collections.abc import Callable
from typing import Any, TypeVar


class HasPerLoopCache:
    def __init__(self) -> None:
        super().__init__()
        self._per_loop_cache: dict[Callable, Any] = {}


S = TypeVar("S", bound=HasPerLoopCache)
T = TypeVar("T")


def cache_per_loop(method: Callable[[S], T]) -> Callable[[S], T]:
    """
    Cache a getter method on a component until the cache is cleared.

    Stores a cache in a dict called ``_per_loop_cache`` on the instance.
    This is expected to be cleared in ``robotPeriodic()``.
    """

    @functools.wraps(method)
    def wrapper(self: S) -> T:
        if method not in self._per_loop_cache:
            self._per_loop_cache[method] = method(self)
        return self._per_loop_cache[method]

    return wrapper
