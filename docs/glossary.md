# AUV Control Glossary

1. **Added mass** - Added mass represents the extra inertia an underwater vehicle experiences because it must also accelerate some of the surrounding water. Without accounting for it, the simulator would predict incorrect accelerations and vehicle responses.

2. **LOS guidance** - Line-of-Sight (LOS) guidance calculates a desired heading (`psi_d`) using the direction of the path and the vehicle's cross-track position error, with a lookahead distance controlling the correction. Without it, the vehicle would lack a systematic way to steer back toward its intended path.

3. **Cross-track error** - Cross-track error measures the shortest horizontal distance (x and y only) between the vehicle's position and the nearest point on the intended path. Without this measurement, we would have no reliable way to quantify how far the vehicle drifts away from its planned route.

4. **Anti-windup** - Anti-windup prevents the PID controller's integral term from accumulating excessively by allowing integration only within specified error zones and limiting the integral value. Without it, large errors cause excessive integral buildup, resulting in overshoot and slow recovery (in this project, about 15% depth overshoot on a 2 m step before the fix).

5. **Saturation** - Saturation occurs when a thruster reaches its maximum available thrust. When that happens, the allocator scales down all thruster commands together to stay within physical limits while preserving the direction of the commanded force and torque. Without this, clipping each thruster separately would distort that direction, and the vehicle would not move the way the controller intended. The trade-off is a slower response during saturation.
