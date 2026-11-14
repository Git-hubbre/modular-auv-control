# Devlog

Newest entries at the bottom. One short entry per working day.

## 2026-11-10 - Day 1

- Built the first version: 6-DOF simulator, 8-thruster allocator, cascaded PID, LOS guidance, 8 unit tests.
- Found and fixed two bugs: integrator windup on big steps (15% depth overshoot), and a missed waypoint switch when current pushed the vehicle 1.0 m off the path.
- Results with current, gusts and noise: depth step settles in 5 s with 0.4% overshoot; heading step settles in 7 s with 4.4% overshoot; waypoint mission cross-track RMS 0.22 m.
- Open issue: the simulator gives the controller x/y position, which cheap hardware will not have. Next big step is removing it and adding an estimator.
  
## 2026-11-12 - Day 3

- Added the CI badge to the README so test status is visible at a glance.
- Next: write docs/frames.md explaining the NED axes in my own words.

## 2026-11-14 - Day 5

- Added an AUV control glossary explaining added mass, LOS guidance, cross-track error, anti-windup and thruster saturation.
