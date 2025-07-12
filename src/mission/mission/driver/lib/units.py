from dataclasses import dataclass


@dataclass(frozen=True)
class Duration:
    s: float

    @property
    def T(self):
        return self

    @property
    def ms(self) -> float:
        return self.s * 1e3

    @property
    def us(self) -> float:
        return self.s * 1e6

    @property
    def ns(self) -> float:
        return self.s * 1e9


Seconds = Duration


class Milliseconds(Duration):
    def __new__(cls, ms: float):
        return Duration(ms * 1e-3)


class Microseconds(Duration):
    def __new__(cls, us: float):
        return Duration(us * 1e-6)


class Nanoseconds(Duration):
    def __new__(cls, ns: float):
        return Duration(ns * 1e-9)


@dataclass(frozen=True)
class Frequency:
    F: float

    @property
    def T(self):
        return Seconds(1 / self.F)

    @property
    def Hz(self) -> float:
        """Return the period in microseconds."""
        return self.F

    @property
    def KHz(self) -> float:
        """Return the period in milliseconds."""
        return self.F * 1e-3

    @property
    def MHz(self) -> float:
        """Return the period in seconds."""
        return self.F * 1e-6

    @property
    def GHz(self) -> float:
        """Return the period in seconds."""
        return self.F * 1e-9


Hz = Frequency


class KHz(Frequency):
    def __new__(cls, KHz: float):
        return Frequency(KHz * 1e3)


class MHz(Frequency):
    def __new__(cls, MHz: float):
        return Frequency(MHz * 1e6)


class GHz(Frequency):
    def __new__(cls, GHz: float):
        return Frequency(GHz * 1e9)
