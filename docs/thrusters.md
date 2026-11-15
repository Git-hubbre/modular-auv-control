# Thruster layout and allocation

## The basic idea

A thruster is a propeller driven by a motor. It pushes water one way, so the vehicle is pushed the other way: a **force** along the thruster's axis. Reversing the motor reverses the force. In this project, thrust is measured in newtons, and a positive value pushes along the thruster's arrow in the diagram below.

A force that acts away from the vehicle's centre also **twists** the vehicle. This twist is called a moment (torque): moment = lever arm (distance from the centre) x force. Pushing a door near its edge swings it, while pushing at the hinge does nothing. So where a thruster sits matters as much as where it points.

## Layout (top view)

```
                 front (+x)
                    ^
   V_FL (o)         |         (o) V_FR
          H_FL /    |    \ H_FR
                    |
   port (-y) -------+-------> starboard (+y)
                    |
          H_RL \    |    / H_RR
   V_RL (o)         |         (o) V_RR
```

- `H_*` (horizontal): four thrusters angled at 45 degrees. `/` pushes the vehicle forward and to the right. `\` pushes it forward and to the left.
- `V_*` (vertical): four thrusters pointing straight up/down, placed near the corners, away from the centre. `(o)` means the axis goes through the page.
- Positive vertical thrust pushes the vehicle down (z is down in our frame).

## What each motion needs

| Motion | Positive means | Thrusters | How |
|---|---|---|---|
| Surge | forward | all 4 horizontal | all push positive |
| Sway | to starboard (right) | H_FL, H_RR positive; H_FR, H_RL negative | the two `/` push, the two `\` pull back |
| Yaw | nose turns right | H_FL, H_RL positive; H_FR, H_RR negative | pushes form a twist |
| Heave | down | all 4 vertical | all push positive |
| Roll | right side goes down | V_FR, V_RR positive; V_FL, V_RL negative | right pushes down, left pushes up |
| Pitch | nose goes up | V_RR, V_RL positive; V_FR, V_FL negative | rear pushes down, front pushes up |

Equal pushes on opposite corners cancel each other's twist. Four equal vertical thrusts give pure heave with no tilt.

## The matrix B

`auv/thrusters.py` builds a 6 x 8 matrix `B` in `default_B()`. Each column is one thruster, and each row is one motion (surge, sway, heave, roll, pitch, yaw). The entry says how much force or moment one newton of that thruster creates in that motion.

```
tau = B @ u
```

- `u` = the 8 thruster forces (what the hardware does)
- `tau` = the 6 resulting forces and moments on the vehicle (what the controller asks for)

**Allocation** works backward: the controller asks for `tau`, and the allocator finds `u` with `u = pinv(B) @ tau`. Worked values from the code:

| Controller asks for | Each thruster used |
|---|---|
| 10 N surge | 3.54 N on each of the 4 horizontal |
| 10 N heave | 2.50 N on each of the 4 vertical |
| 1 Nm yaw | 1.47 N on each horizontal (signs alternate) |
| 1 Nm roll | 1.14 N on each vertical |
| 1 Nm pitch | 2.08 N on each vertical |

Pitch needs more force per newton-metre than roll because the vertical thrusters are only 0.12 m from the centre fore-aft but 0.22 m sideways. A longer lever arm means less force is needed.

## Why eight thrusters

Six motions need at least six thrusters, so eight gives two spare. The matrix has rank 6 (`test_allocation_matrix_full_rank`), which means every motion can be commanded. With extra thrusters there are many ways to make the same `tau`. `pinv` picks the solution with the smallest total thrust, which shares the work evenly. The spare capacity is also what would let a real vehicle keep working after losing a thruster.

## Saturation

Each thruster has a limit (`max_thrust`, 40 N here). If the controller demands more, the allocator scales all eight thrusters down by the same factor. Clipping each one separately would change the mix and push the vehicle in a different direction than intended.

## Simplifications in this model

- Thrust is assumed linear and equally strong forward and backward. Real thrusters are weaker in reverse.
- Thrusters sit in one plane through the centre of gravity, so a vertical offset adds no moment.
- Thruster-to-thruster water interference is ignored.
- Motor lag is modelled as a simple first-order delay (0.15 s).
