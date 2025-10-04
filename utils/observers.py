"""Observable helpers for sharing runtime state between modules."""

from __future__ import annotations

import threading
from typing import Callable, Generic, List, TypeVar

T = TypeVar("T")


class ObservableValue(Generic[T]):
    """Thread-safe observable value container."""

    def __init__(self, initial: T) -> None:
        self._value = initial
        self._lock = threading.Lock()
        self._observers: List[Callable[[T], None]] = []

    def subscribe(
        self,
        callback: Callable[[T], None],
        *,
        notify_immediately: bool = True,
    ) -> Callable[[], None]:
        """Register *callback* and return an unsubscribe function."""

        with self._lock:
            self._observers.append(callback)
            current = self._value

        if notify_immediately:
            try:
                callback(current)
            except Exception:
                # Observers should handle their own errors; ignore to keep others notified.
                pass

        def _unsubscribe() -> None:
            self.unsubscribe(callback)

        return _unsubscribe

    def unsubscribe(self, callback: Callable[[T], None]) -> None:
        with self._lock:
            self._observers = [cb for cb in self._observers if cb is not callback]

    def set(self, value: T) -> None:
        with self._lock:
            self._value = value
            observers = list(self._observers)

        for cb in observers:
            try:
                cb(value)
            except Exception:
                pass

    def get(self) -> T:
        with self._lock:
            return self._value

    @property
    def value(self) -> T:
        return self.get()


class ObservableFloat(ObservableValue[float]):
    """Observable that normalises values to ``float`` before broadcasting."""

    def set(self, value: float) -> None:  # type: ignore[override]
        super().set(float(value))


__all__ = ["ObservableValue", "ObservableFloat"]
