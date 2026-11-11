"""The hardware boundary.

The controller only ever sees a `Measurement` and only ever calls `send_thrust`.
The simulator implements this interface today; a real vehicle driver (IMU + pressure
sensor + ESC outputs on an ESP32 / Raspberry Pi) implements the same two methods later.
Nothing in controller.py, guidance.py or thrusters.py needs to change.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class Measurement:
    t: float            # time [s]
    eta: np.ndarray     # [x, y, z, roll, pitch, yaw]  (m, rad), world NED
    nu: np.ndarray      # [u, v, w, p, q, r]           (m/s, rad/s), body frame


@dataclass
class Reference:
    depth: float        # desired z [m] (positive down)
    heading: float      # desired yaw [rad]
    surge: float = 0.0  # desired body surge speed [m/s]
    sway: float = 0.0   # desired body sway speed [m/s]


class VehicleInterface(ABC):
    dt: float

    @abstractmethod
    def read_state(self) -> Measurement:
        """Return the latest (possibly noisy) state estimate."""

    @abstractmethod
    def send_thrust(self, thrust_cmd: np.ndarray) -> None:
        """Send per-thruster force commands in newtons."""

    @abstractmethod
    def advance(self) -> None:
        """Sim: integrate one step. Hardware: sleep until the next control tick."""
