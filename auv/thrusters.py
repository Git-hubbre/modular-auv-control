"""Thruster geometry and allocation (generalized force/torque -> per-thruster thrust).

BlueROV2-Heavy-style layout: 4 vectored horizontal thrusters (surge, sway, yaw) and
4 vertical thrusters (heave, roll, pitch). Gives full 6-DOF control authority.
"""
from dataclasses import dataclass, field

import numpy as np

NAMES = ["H_FR", "H_FL", "H_RR", "H_RL", "V_FR", "V_FL", "V_RR", "V_RL"]


def _column(pos, direction):
    pos = np.asarray(pos, float)
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    return np.concatenate([d, np.cross(pos, d)])  # [force; moment about CG]


def default_B():
    cols = [
        _column((0.14, 0.10, 0.0), (1, -1, 0)),   # H_FR
        _column((0.14, -0.10, 0.0), (1, 1, 0)),   # H_FL
        _column((-0.14, 0.10, 0.0), (1, 1, 0)),   # H_RR
        _column((-0.14, -0.10, 0.0), (1, -1, 0)), # H_RL
        _column((0.12, 0.22, 0.0), (0, 0, 1)),    # V_FR
        _column((0.12, -0.22, 0.0), (0, 0, 1)),   # V_FL
        _column((-0.12, 0.22, 0.0), (0, 0, 1)),   # V_RR
        _column((-0.12, -0.22, 0.0), (0, 0, 1)),  # V_RL
    ]
    return np.array(cols).T  # 6 x 8


@dataclass
class ThrusterConfig:
    max_thrust: float = 40.0   # N per thruster (each direction)
    time_constant: float = 0.15  # s, first-order thruster lag
    B: np.ndarray = field(default_factory=default_B)

    def __post_init__(self):
        self.B_pinv = np.linalg.pinv(self.B)

    @property
    def n(self):
        return self.B.shape[1]

    def allocate(self, tau_cmd):
        """Minimum-norm allocation. If any thruster saturates, scale all thrusters
        down together so the direction of the commanded force/torque is preserved."""
        u = self.B_pinv @ np.asarray(tau_cmd, float)
        peak = np.max(np.abs(u))
        if peak > self.max_thrust:
            u = u * (self.max_thrust / peak)
        return u
