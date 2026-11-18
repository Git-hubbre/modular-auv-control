# Gain tuning

I changed one gain at a time (x0.5 and x2) and re-ran the same step test as the demo: depth 0.5 -> 2.5 m and heading 0 -> 60 deg at t = 1 s, with ocean current, gusts and sensor noise (seed 1).

Default gains: depth (kp, ki, kd) = (60, 8, 70), heading = (4.0, 0.8, 3.0).

| Variant | Depth overshoot % | Depth settle s | Heading overshoot % | Heading settle s | Heading steady-state error (deg) | Peak thruster N |
|---|---|---|---|---|---|---|
| baseline | 0.4 | 5.0 | 4.4 | 7.1 | 0.42 | 30.6 |
| depth kp x0.5 | 2.9 | 16.0 | - | - | - | 15.8 |
| depth kp x2 | 0.3 | 2.4 | - | - | - | 40.0 |
| depth ki x0.5 | 0.1 | 5.2 | - | - | - | 30.6 |
| depth ki x2 | 1.6 | 4.7 | - | - | - | 30.6 |
| depth kd x0.5 | 2.3 | 3.9 | - | - | - | 30.7 |
| depth kd x2 | 1.8 | 8.9 | - | - | - | 30.7 |
| heading kp x0.5 | - | - | 8.4 | 26.4 | 0.72 | 30.6 |
| heading kp x2 | - | - | 1.9 | 1.5 | 0.15 | 30.6 |
| heading ki x0.5 | - | - | 3.1 | 6.9 | 0.24 | 30.6 |
| heading ki x2 | - | - | 6.3 | 6.3 | 0.32 | 30.6 |
| heading kd x0.5 | - | - | 3.2 | 4.6 | 0.40 | 30.6 |
| heading kd x2 | - | - | 5.4 | 10.5 | 0.44 | 30.6 |

"-" means that axis did not change from the baseline.

## What the results show

- The depth and heading loops do not affect each other. Changing depth gains left the heading numbers identical, and the reverse is also true.
- Doubling depth kp makes the response twice as fast (2.4 s) but pushes a thruster to its 40 N limit, so faster is not free.
- Halving depth kp is too sluggish. It takes 16 s to settle.
- More depth ki or kd did not help. Both made the response worse than baseline.
- Heading kp x2 beat my baseline on every measure: 1.9% overshoot, 1.5 s settling, 0.15 deg steady-state error. My baseline heading gains were chosen conservatively after the windup bug, so there is room to improve them.
- Heading kp x0.5 is clearly bad: 8.4% overshoot and 26 s to settle.

## Limits of this test

- One noise seed. Differences of about 1 s in settling time are not meaningful; only the large effects above are.
- Settling time depends on the band chosen, so compare variants within this table rather than against other projects.
- Only one gain changed at a time, so interactions between gains were not tested.

## Next

Try heading kp x2 as the new default, re-run the tests and the waypoint mission, and check that cross-track error does not get worse.
