"""6-DOF underwater vehicle simulator (Fossen-style) implementing VehicleInterface.

    M nu_r_dot + C(nu_r) nu_r + D(nu_r) nu_r + g(eta) = tau

with nu_r = nu - nu_current (velocity relative to the water). Parameters are
BlueROV2-class (approximate, from published identification work) -- good enough for
control design practice, NOT a validated model of any specific vehicle.
"""
from dataclasses import dataclass

import numpy as np

from .frames import euler_rate_matrix, rot_zyx
from .interfaces import Measurement, VehicleInterface
from .thrusters import ThrusterConfig


@dataclass
class VehicleParams:
    m: float = 11.5
    W: float = 112.8                # weight [N]
    B: float = 114.8                # buoyancy [N]  (2 N net positive buoyancy)
    inertia: tuple = (0.16, 0.16, 0.16)
    added: tuple = (5.5, 12.7, 14.57, 0.12, 0.12, 0.12)
    lin_damp: tuple = (4.03, 6.22, 5.18, 0.07, 0.07, 0.07)
    quad_damp: tuple = (18.18, 21.66, 36.99, 1.55, 1.55, 1.55)
    r_b: tuple = (0.0, 0.0, -0.02)  # centre of buoyancy relative to CG (above CG)

    def mass_diag(self):
        rb = np.array([self.m] * 3 + list(self.inertia))
        return rb + np.array(self.added)


def restoring(params, eta):
    """Gravity/buoyancy generalized force g(eta), CG at body origin."""
    phi, theta, _ = eta[3:]
    xb, yb, zb = params.r_b
    W, B = params.W, params.B
    s, c = np.sin, np.cos
    return np.array([
        (W - B) * s(theta),
        -(W - B) * c(theta) * s(phi),
        -(W - B) * c(theta) * c(phi),
        yb * B * c(theta) * c(phi) - zb * B * c(theta) * s(phi),
        -zb * B * s(theta) - xb * B * c(theta) * c(phi),
        xb * B * c(theta) * s(phi) + yb * B * s(theta),
    ])


class SimulatedVehicle(VehicleInterface):
    def __init__(self, params=None, thrusters=None, dt=0.02, current=(0.0, 0.0, 0.0),
                 gust_amp=0.0, noise=True, seed=0, eta0=None):
        self.p = params or VehicleParams()
        self.thr = thrusters or ThrusterConfig()
        self.dt = dt
        self.current0 = np.array(current, float)
        self.gust_amp = gust_amp
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.M = self.p.mass_diag()
        self.lin = np.array(self.p.lin_damp)
        self.quad = np.array(self.p.quad_damp)
        eta0 = np.zeros(6) if eta0 is None else np.array(eta0, float)
        self.x = np.concatenate([eta0, np.zeros(6), np.zeros(self.thr.n)])
        self.cmd = np.zeros(self.thr.n)
        self.t = 0.0
        # measurement noise std: [x y z], [roll pitch yaw], [u v w], [p q r]
        self.sigma = np.array([0.05, 0.05, 0.01, 0.003, 0.003, 0.005,
                               0.01, 0.01, 0.01, 0.005, 0.005, 0.005])

    # ---- environment -------------------------------------------------------
    def current(self, t):
        """World-frame water current velocity and its time derivative."""
        a = self.gust_amp
        vc = self.current0 + a * np.array([np.sin(0.3 * t), np.cos(0.2 * t), 0.0])
        vcd = a * np.array([0.3 * np.cos(0.3 * t), -0.2 * np.sin(0.2 * t), 0.0])
        return vc, vcd

    # ---- dynamics ----------------------------------------------------------
    def _deriv(self, x, t, cmd):
        eta, nu, thr = x[:6], x[6:12], x[12:]
        phi, theta, psi = eta[3:]
        R = rot_zyx(phi, theta, psi)
        T = euler_rate_matrix(phi, theta)
        vc, vcd = self.current(t)
        vc_b = R.T @ vc
        nu_r = nu.copy()
        nu_r[:3] -= vc_b
        v, w = nu_r[:3], nu_r[3:]
        Mv, Iw = self.M[:3] * v, self.M[3:] * w
        C = np.concatenate([np.cross(w, Mv), np.cross(v, Mv) + np.cross(w, Iw)])
        D = (self.lin + self.quad * np.abs(nu_r)) * nu_r
        tau = self.thr.B @ thr
        nu_r_dot = (tau - C - D - restoring(self.p, eta)) / self.M
        nu_dot = nu_r_dot.copy()
        # nu = nu_r + nu_c, with nu_c = R^T vc rotating with the body
        nu_dot[:3] += -np.cross(nu[3:], vc_b) + R.T @ vcd
        eta_dot = np.concatenate([R @ nu[:3], T @ nu[3:]])
        thr_dot = (cmd - thr) / self.thr.time_constant
        return np.concatenate([eta_dot, nu_dot, thr_dot])

    def advance(self):
        h, t, x, u = self.dt, self.t, self.x, self.cmd
        k1 = self._deriv(x, t, u)
        k2 = self._deriv(x + 0.5 * h * k1, t + 0.5 * h, u)
        k3 = self._deriv(x + 0.5 * h * k2, t + 0.5 * h, u)
        k4 = self._deriv(x + h * k3, t + h, u)
        self.x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.t += h

    # ---- interface ---------------------------------------------------------
    def send_thrust(self, thrust_cmd):
        self.cmd = np.clip(thrust_cmd, -self.thr.max_thrust, self.thr.max_thrust)

    def read_state(self):
        z = self.x[:12].copy()
        if self.noise:
            z = z + self.rng.normal(0.0, self.sigma)
        return Measurement(t=self.t, eta=z[:6], nu=z[6:])

    @property
    def true_eta(self):
        return self.x[:6].copy()

    @property
    def true_nu(self):
        return self.x[6:12].copy()

    @property
    def true_thrust(self):
        return self.x[12:].copy()
