"""Line-of-sight waypoint guidance: Measurement -> (Reference, done)."""
import numpy as np

from .interfaces import Measurement, Reference


class LOSGuidance:
    def __init__(self, waypoints, cruise=0.5, lookahead=2.0, accept=1.0, finish_radius=0.5):
        self.waypoints = [np.array(w, float) for w in waypoints]
        self.cruise, self.lookahead = cruise, lookahead
        self.accept, self.finish_radius = accept, finish_radius
        self.path = None       # (N+1, 3) once started: start point + waypoints
        self.i = 0             # current segment index
        self.last_heading = 0.0

    def _segment(self, pos):
        p0, p1 = self.path[self.i], self.path[self.i + 1]
        seg = p1[:2] - p0[:2]
        L = np.linalg.norm(seg)
        d = pos[:2] - p0[:2]
        along = d @ seg / L
        dist = np.linalg.norm(p1[:2] - pos[:2])
        return p0, p1, seg, L, d, along, dist

    def update(self, meas: Measurement):
        pos = meas.eta[:3]
        if self.path is None:
            self.path = np.vstack([pos, *self.waypoints])
            self.last_heading = meas.eta[5]
        n_seg = len(self.path) - 1

        # Advance when within the acceptance radius OR when we have passed the waypoint
        # along-track (robust to a persistent cross-track offset from currents).
        while True:
            p0, p1, seg, L, d, along, dist = self._segment(pos)
            last = self.i == n_seg - 1
            if last:
                if dist < self.finish_radius or along > L:
                    return Reference(depth=p1[2], heading=self.last_heading), True
                break
            if dist < self.accept or along > L:
                self.i += 1
                continue
            break

        alpha = np.arctan2(seg[1], seg[0])
        cross = -d[0] * np.sin(alpha) + d[1] * np.cos(alpha)
        psi_d = alpha + np.arctan2(-cross, self.lookahead)
        self.last_heading = psi_d

        err = (psi_d - meas.eta[5] + np.pi) % (2 * np.pi) - np.pi
        speed = self.cruise * max(0.0, np.cos(err))
        if last:
            speed = min(speed, 0.5 * dist + 0.05)
        z_d = p0[2] + (p1[2] - p0[2]) * np.clip(along / L, 0.0, 1.0)
        return Reference(depth=z_d, heading=psi_d, surge=speed, sway=0.0), False
