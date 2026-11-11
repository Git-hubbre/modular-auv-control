"""Frame transforms (NED world frame, z down; body frame x fwd, y right, z down)."""
import numpy as np


def wrap(angle):
    """Wrap an angle (rad) to [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rot_zyx(phi, theta, psi):
    """Rotation matrix body -> world for ZYX Euler angles (roll, pitch, yaw)."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    return np.array([
        [cpsi * cth, -spsi * cphi + cpsi * sth * sphi, spsi * sphi + cpsi * cphi * sth],
        [spsi * cth, cpsi * cphi + sphi * sth * spsi, -cpsi * sphi + sth * spsi * cphi],
        [-sth, cth * sphi, cth * cphi],
    ])


def euler_rate_matrix(phi, theta):
    """Maps body angular rates (p,q,r) to Euler angle rates. Singular at theta = +/-90 deg."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, tth = np.cos(theta), np.tan(theta)
    return np.array([
        [1.0, sphi * tth, cphi * tth],
        [0.0, cphi, -sphi],
        [0.0, sphi / cth, cphi / cth],
    ])
