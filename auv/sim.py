"""Closed-loop runner and performance metrics. Works with any VehicleInterface."""
import numpy as np


def run(vehicle, controller, allocator, ref_fn, t_max):
    """ref_fn(measurement) -> (Reference, done). Returns a dict of logged arrays."""
    log = {k: [] for k in ["t", "eta", "nu", "ref_depth", "ref_heading", "ref_surge",
                           "tau", "u_cmd", "u_act"]}
    dt = vehicle.dt
    while vehicle.t < t_max:
        meas = vehicle.read_state()
        ref, done = ref_fn(meas)
        tau = controller.update(meas, ref, dt)
        u = allocator.allocate(tau)
        vehicle.send_thrust(u)
        log["t"].append(vehicle.t)
        log["eta"].append(vehicle.true_eta)
        log["nu"].append(vehicle.true_nu)
        log["ref_depth"].append(ref.depth)
        log["ref_heading"].append(ref.heading)
        log["ref_surge"].append(ref.surge)
        log["tau"].append(tau)
        log["u_cmd"].append(u)
        log["u_act"].append(vehicle.true_thrust)
        if done:
            break
        vehicle.advance()
    return {k: np.array(v) for k, v in log.items()}


def step_metrics(t, y, y0, y_final, t_step):
    """Rise time (10-90%), overshoot %, 2% settling time, steady-state error."""
    m = t >= t_step
    tt, yy = t[m] - t_step, y[m]
    norm = (yy - y0) / (y_final - y0)
    t10 = tt[np.argmax(norm >= 0.1)]
    t90 = tt[np.argmax(norm >= 0.9)]
    outside = np.where(np.abs(norm - 1.0) > 0.02)[0]
    settle = tt[min(outside[-1] + 1, len(tt) - 1)] if len(outside) else 0.0
    tail = tt >= tt[-1] - 5.0
    return {
        "rise_time_s": float(t90 - t10),
        "overshoot_pct": float(max(0.0, norm.max() - 1.0) * 100.0),
        "settling_time_s": float(settle),
        "steady_state_error": float(np.mean(y_final - yy[tail])),
    }


def cross_track_error(xy, path_xy):
    """Distance from each (x, y) sample to the nearest segment of the polyline path."""
    out = np.full(len(xy), np.inf)
    for a, b in zip(path_xy[:-1], path_xy[1:]):
        ab = b - a
        s = np.clip(((xy - a) @ ab) / (ab @ ab), 0.0, 1.0)
        proj = a + s[:, None] * ab
        out = np.minimum(out, np.linalg.norm(xy - proj, axis=1))
    return out
