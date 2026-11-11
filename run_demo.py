"""Runs both scenarios, writes plots + metrics.json into results/.

    python run_demo.py
"""
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from auv.controller import VehicleController
from auv.guidance import LOSGuidance
from auv.interfaces import Reference
from auv.sim import cross_track_error, run, step_metrics
from auv.thrusters import NAMES, ThrusterConfig
from auv.vehicle import SimulatedVehicle

OUT = Path(__file__).parent / "results"
OUT.mkdir(exist_ok=True)
CURRENT = (0.15, -0.10, 0.0)   # m/s, world frame (~0.18 m/s)
GUST = 0.05


def step_scenario():
    z0, z1, psi1, t_step = 0.5, 2.5, np.radians(60.0), 1.0
    veh = SimulatedVehicle(current=CURRENT, gust_amp=GUST, seed=1, eta0=[0, 0, z0, 0, 0, 0])

    def ref_fn(m):
        on = m.t >= t_step
        return Reference(depth=z1 if on else z0, heading=psi1 if on else 0.0), False

    log = run(veh, VehicleController(), ThrusterConfig(), ref_fn, 40.0)
    t = log["t"]
    heading_deg = np.degrees(np.unwrap(log["eta"][:, 5]))
    metrics = {
        "depth_step_0.5m_to_2.5m": step_metrics(t, log["eta"][:, 2], z0, z1, t_step),
        "heading_step_0_to_60deg": step_metrics(t, heading_deg, 0.0, 60.0, t_step),
        "peak_roll_pitch_deg": float(np.degrees(np.max(np.abs(log["eta"][:, 3:5])))),
        "peak_thruster_cmd_N": float(np.max(np.abs(log["u_cmd"]))),
    }

    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    ax[0, 0].plot(t, log["ref_depth"], "k--", label="reference")
    ax[0, 0].plot(t, log["eta"][:, 2], label="depth")
    ax[0, 0].set(ylabel="depth [m]", title="Depth step")
    ax[0, 1].plot(t, np.degrees(log["ref_heading"]), "k--", label="reference")
    ax[0, 1].plot(t, heading_deg, label="heading")
    ax[0, 1].set(ylabel="heading [deg]", title="Heading step")
    ax[1, 0].plot(t, np.degrees(log["eta"][:, 3]), label="roll")
    ax[1, 0].plot(t, np.degrees(log["eta"][:, 4]), label="pitch")
    ax[1, 0].set(ylabel="angle [deg]", xlabel="time [s]", title="Attitude (should stay near 0)")
    for i, n in enumerate(NAMES):
        ax[1, 1].plot(t, log["u_act"][:, i], label=n, lw=1)
    ax[1, 1].set(ylabel="thrust [N]", xlabel="time [s]", title="Thruster forces")
    ax[1, 1].legend(ncol=2, fontsize=7)
    for a in ax.flat[:3]:
        a.legend()
        a.grid(alpha=0.3)
    ax[1, 1].grid(alpha=0.3)
    fig.suptitle("Step response with current (0.18 m/s) + gusts + sensor noise")
    fig.tight_layout()
    fig.savefig(OUT / "step_response.png", dpi=130)
    plt.close(fig)
    return metrics


def waypoint_scenario(current, gust):
    wps = [(8, 0, 2.0), (8, 8, 2.0), (0, 8, 3.0), (0, 0, 3.0)]
    veh = SimulatedVehicle(current=current, gust_amp=gust, seed=2, eta0=[0, 0, 0.5, 0, 0, 0])
    guid = LOSGuidance(wps, cruise=0.5, lookahead=2.0)
    log = run(veh, VehicleController(), ThrusterConfig(), guid.update, 250.0)
    xy = log["eta"][:, :2]
    cte = cross_track_error(xy, guid.path[:, :2])
    return log, guid.path, cte


def waypoint_all():
    log_c, path, cte_c = waypoint_scenario(CURRENT, GUST)
    log_n, _, cte_n = waypoint_scenario((0, 0, 0), 0.0)

    def summary(log, cte):
        return {
            "mission_time_s": float(log["t"][-1]),
            "cross_track_rmse_m": float(np.sqrt(np.mean(cte ** 2))),
            "cross_track_max_m": float(cte.max()),
            "depth_rmse_vs_ref_m": float(np.sqrt(np.mean((log["eta"][:, 2] - log["ref_depth"]) ** 2))),
        }

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.8))
    ax[0].plot(path[:, 1], path[:, 0], "k--", marker="o", label="planned path")
    ax[0].plot(log_n["eta"][:, 1], log_n["eta"][:, 0], label="no current")
    ax[0].plot(log_c["eta"][:, 1], log_c["eta"][:, 0], label="with current")
    ax[0].set(xlabel="East y [m]", ylabel="North x [m]", title="Path following (top view)")
    ax[0].axis("equal")
    ax[1].plot(log_n["t"], cte_n, label="no current")
    ax[1].plot(log_c["t"], cte_c, label="with current")
    ax[1].set(xlabel="time [s]", ylabel="cross-track error [m]", title="Cross-track error")
    ax[2].plot(log_c["t"], log_c["ref_depth"], "k--", label="reference")
    ax[2].plot(log_c["t"], log_c["eta"][:, 2], label="depth (with current)")
    ax[2].invert_yaxis()
    ax[2].set(xlabel="time [s]", ylabel="depth [m]", title="Depth along mission")
    for a in ax:
        a.legend()
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUT / "waypoint_mission.png", dpi=130)
    plt.close(fig)
    return {"with_current": summary(log_c, cte_c), "no_current": summary(log_n, cte_n)}


if __name__ == "__main__":
    metrics = {"step_response": step_scenario(), "waypoint_mission": waypoint_all()}
    (OUT / "metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
