# Coordinate frames and sign conventions

Every number in this project lives in one of two frames. Mixing them up is the most common source of bugs in vehicle control, so this page pins them down.

## World frame: NED

- **x = North**, **y = East**, **z = Down**
- Depth is simply `z`, so **depth is positive downward**. Rising means `z` decreases.
- Position vector: `eta[0:3] = [x, y, z]`

## Body frame (fixed to the vehicle)

- **x = forward (surge)**, **y = right/starboard (sway)**, **z = down (heave)**
- Velocity vector: `nu[0:3] = [u, v, w]` in m/s, measured along the body axes
- Angular rates: `nu[3:6] = [p, q, r]` about the body x, y, z axes

```
        x (forward)
          ^
          |
  port    |    starboard
 (left)   +----> y (right)
         /
        v  z (down, into the page)
```

## Angles (right-hand rule about each body axis)

| Angle | Axis | Positive means |
|---|---|---|
| roll (phi) | x | starboard side goes down |
| pitch (theta) | y | nose goes up |
| yaw (psi) | z | nose turns right (clockwise seen from above) |

Heading is yaw: 0 = facing North, +90 deg = facing East.

## Converting between frames

`auv/frames.py` builds the rotation matrix `R = rot_zyx(phi, theta, psi)`:

- `R @ v_body` gives that vector in world axes. The velocity part of the state derivative uses this.
- `R.T @ v_world` goes the other way. The simulator uses it to express the water current in body axes, and the controller uses it to turn a world-vertical force into body-frame forces.

Check by hand: with yaw = 90 deg and no roll or pitch, `R @ [1, 0, 0]` is `[0, 1, 0]`. A vehicle facing East moving forward moves East.

## Things that surprise people

1. **Buoyancy up is negative `w` and negative `z` velocity.** The vehicle has +2 N net buoyancy, so at rest it floats up: `z` decreases. A unit test checks this.
2. **Euler angles break at pitch = +/-90 deg** (gimbal lock). `euler_rate_matrix` divides by cos(theta). Our vehicle never pitches that far, but a real flip would need quaternions.
3. **Yaw wraps.** 179 deg and -179 deg are only 2 deg apart. `wrap()` in `auv/frames.py` keeps errors in [-pi, pi). Without it the vehicle would spin the long way round.
4. **Depth control happens in the world frame, speed control in the body frame.** That is why `controller.py` rotates the depth force with `R.T`.

## Exercise

In `tests/test_core.py`, add a test that checks the yaw = 90 deg example above. If it passes, you understand the frames.
