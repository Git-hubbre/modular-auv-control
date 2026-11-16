# Architecture

The stack follows the guidance, navigation and control (GNC) split used in marine robotics. Each part talks to the others only through small data objects, so any part can be replaced without touching the rest.

## Data flow (one control step)

```
 Vehicle (SimulatedVehicle now, real hardware later)
   |  read_state()
   v
 Measurement  (t, eta = position and angles, nu = velocities)
   |
   +--> Guidance (LOS) -----> Reference (depth, heading, surge, sway)
   |                              |
   +-------> Controller <---------+      (cascaded PID; also uses the measurement as feedback)
                 |  tau: 6 forces and moments
                 v
             Allocator (pseudo-inverse + saturation scaling)
                 |  u: 8 thruster forces [N]
                 v
 Vehicle <-- send_thrust(u), then advance()
```

`run()` in `auv/sim.py` repeats this loop every time step and logs the results.

## GNC mapping

| Part | File | Status |
|---|---|---|
| Guidance: decides where to go (heading and speed from waypoints) | `auv/guidance.py` | done |
| Navigation: estimates where the vehicle is from noisy sensors | planned | not started |
| Control: turns the reference into forces, then thruster commands | `auv/controller.py`, `auv/thrusters.py` | done |

Navigation is the missing piece. At the moment the controller reads the sensor values directly, and the simulator even provides x/y position, which a low-cost vehicle will not have. The estimator will sit between `read_state()` and guidance/controller, taking raw measurements in and giving a cleaner estimate of the same shape out.

## The hardware boundary

`auv/interfaces.py` defines `VehicleInterface` with three methods: `read_state()`, `send_thrust()` and `advance()`. The simulator implements it today. A real vehicle driver (IMU, pressure sensor, motor controllers) would implement the same three methods later. Guidance, controller and allocator never import the simulator, so none of them needs to change.

## Files

| File | Role |
|---|---|
| `auv/vehicle.py` | 6-DOF dynamics, current, thruster lag, noisy sensors |
| `auv/thrusters.py` | 8-thruster geometry and allocation (see `docs/thrusters.md`) |
| `auv/controller.py` | depth, heading, attitude and speed loops |
| `auv/guidance.py` | line-of-sight waypoint following |
| `auv/frames.py` | rotation matrices and angle wrapping (see `docs/frames.md`) |
| `auv/sim.py` | closed-loop runner and metrics |
| `run_demo.py` | scenarios, plots and `metrics.json` |
