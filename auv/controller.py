"""Cascaded PID controller: Measurement + Reference -> generalized force/torque tau (6,).

Derivative terms use measured velocities instead of differentiating noisy positions.
Knows nothing about the simulator or the thrusters.
"""
from dataclasses import dataclass, field

import numpy as np

from .frames import euler_rate_matrix, rot_zyx, wrap
from .interfaces import Measurement, Reference


@dataclass
class Gains:
    depth: tuple = (60.0, 8.0, 70.0)      # kp, ki, kd   [N/m, N/(m s), N s/m]
    heading: tuple = (4.0, 0.8, 3.0)      # kp, ki, kd   [Nm/rad, ...]
    attitude: tuple = (4.0, 1.5)          # kp, kd  (roll & pitch leveling)
    surge: tuple = (40.0, 10.0)           # kp, ki  [N s/m]
    sway: tuple = (40.0, 10.0)
    i_limit: dict = field(default_factory=lambda: {"depth": 40.0, "heading": 1.5, "speed": 20.0})
    # integrate only when |error| is inside this zone (m, rad)
    i_zone: dict = field(default_factory=lambda: {"depth": 0.3, "heading": np.radians(20.0)})


class VehicleController:
    def __init__(self, gains=None):
        self.g = gains or Gains()
        self.reset()

    def reset(self):
        self.i_depth = self.i_head = self.i_surge = self.i_sway = 0.0

    @staticmethod
    def _clamp(x, lim):
        return float(np.clip(x, -lim, lim))

    def update(self, meas: Measurement, ref: Reference, dt: float) -> np.ndarray:
        g, lim = self.g, self.g.i_limit
        phi, theta, psi = meas.eta[3:]
        R = rot_zyx(phi, theta, psi)
        z_dot = (R @ meas.nu[:3])[2]
        psi_dot = (euler_rate_matrix(phi, theta) @ meas.nu[3:])[2]

        # depth (world z) -> force along world z
        e_z = ref.depth - meas.eta[2]
        if abs(e_z) < g.i_zone["depth"]:  # conditional integration: no windup on big steps
            self.i_depth = self._clamp(self.i_depth + g.depth[1] * e_z * dt, lim["depth"])
        f_z = g.depth[0] * e_z + self.i_depth - g.depth[2] * z_dot

        # heading -> yaw moment
        e_psi = wrap(ref.heading - psi)
        if abs(e_psi) < g.i_zone["heading"]:
            self.i_head = self._clamp(self.i_head + g.heading[1] * e_psi * dt, lim["heading"])
        n = g.heading[0] * e_psi + self.i_head - g.heading[2] * psi_dot

        # roll / pitch leveling (PD)
        k = -g.attitude[0] * phi - g.attitude[1] * meas.nu[3]
        m = -g.attitude[0] * theta - g.attitude[1] * meas.nu[4]

        # body-frame speed loops (PI)
        e_u = ref.surge - meas.nu[0]
        e_v = ref.sway - meas.nu[1]
        self.i_surge = self._clamp(self.i_surge + g.surge[1] * e_u * dt, lim["speed"])
        self.i_sway = self._clamp(self.i_sway + g.sway[1] * e_v * dt, lim["speed"])
        f_x = g.surge[0] * e_u + self.i_surge
        f_y = g.sway[0] * e_v + self.i_sway

        force = np.array([f_x, f_y, 0.0]) + R.T @ np.array([0.0, 0.0, f_z])
        return np.concatenate([force, [k, m, n]])
