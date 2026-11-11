import numpy as np

from auv.controller import VehicleController
from auv.frames import rot_zyx, wrap
from auv.interfaces import Reference
from auv.sim import run
from auv.thrusters import ThrusterConfig
from auv.vehicle import SimulatedVehicle, VehicleParams, restoring


def test_allocation_matrix_full_rank():
    assert np.linalg.matrix_rank(ThrusterConfig().B) == 6


def test_allocation_reproduces_request_when_unsaturated():
    cfg = ThrusterConfig()
    tau = np.array([10.0, -5.0, 20.0, 0.5, -0.3, 1.0])
    assert np.allclose(cfg.B @ cfg.allocate(tau), tau, atol=1e-9)


def test_saturation_scaling_preserves_direction():
    cfg = ThrusterConfig()
    tau = np.array([500.0, 200.0, -300.0, 5.0, 5.0, 20.0])
    u = cfg.allocate(tau)
    assert np.max(np.abs(u)) <= cfg.max_thrust + 1e-9
    out = cfg.B @ u
    assert np.allclose(out / np.linalg.norm(out), tau / np.linalg.norm(tau), atol=1e-9)


def test_rotation_is_orthonormal_and_wrap():
    R = rot_zyx(0.3, -0.2, 1.1)
    assert np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)
    assert np.isclose(wrap(3 * np.pi / 2), -np.pi / 2)


def test_restoring_moment_is_stabilizing():
    p = VehicleParams()
    g_roll = restoring(p, np.array([0, 0, 0, 0.1, 0, 0]))
    g_pitch = restoring(p, np.array([0, 0, 0, 0, 0.1, 0]))
    assert g_roll[3] > 0 and g_pitch[4] > 0  # M nu_dot = -g  ->  opposes the tilt


def test_net_buoyancy_floats_up_at_rest():
    v = SimulatedVehicle(noise=False)
    v.advance()
    assert v.true_nu[2] < 0  # z is down, so negative w = rising
    assert np.isclose(v.true_nu[2] / v.dt, -2.0 / VehicleParams().mass_diag()[2], rtol=1e-2)


def test_free_decay_loses_energy():
    v = SimulatedVehicle(noise=False)
    v.x[6:9] = [1.0, 0.5, 0.0]
    v0 = np.linalg.norm(v.true_nu[:3])
    for _ in range(200):
        v.advance()
    assert np.linalg.norm(v.true_nu[:3]) < v0


def test_depth_hold_converges_with_current_and_noise():
    v = SimulatedVehicle(current=(0.2, -0.1, 0.0), gust_amp=0.05, seed=1, eta0=[0, 0, 0.5, 0, 0, 0])
    log = run(v, VehicleController(), ThrusterConfig(),
              lambda m: (Reference(depth=2.0, heading=0.0), False), 30.0)
    assert abs(log["eta"][-1, 2] - 2.0) < 0.05
    assert np.max(np.abs(log["eta"][:, 3:5])) < np.radians(10)
