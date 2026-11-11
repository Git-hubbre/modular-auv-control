# Daily task backlog

One small, real improvement per day. Each task is 15-45 minutes and ends in a commit.
Tick the box in the same commit that finishes the task.

- 📱 = doable from a phone in the GitHub web editor (the pencil icon). CI runs the tests on every push, so you still get a pass/fail check.
- 💻 = needs you to run code locally or in a Codespace.

Commit message style: `area: what changed` (example: `docs: add frames diagram`).

## Week 1 - make the repo look and read well

- [ ] 📱 Add the CI badge to the top of `README.md` (replace `YOU` if your username differs): `![tests](https://github.com/YOU/modular-auv-control/actions/workflows/tests.yml/badge.svg)`
- [ ] 📱 Create `docs/frames.md`: explain NED world axes (x north, y east, z down), body axes, and why depth is positive down.
- [ ] 📱 Create `docs/glossary.md`: added mass, Munk moment, LOS guidance, cross-track error, anti-windup, in your own words.
- [ ] 📱 Create `docs/thrusters.md`: draw the 8-thruster layout as ASCII art and say which thrusters give surge, sway, heave, roll, pitch, yaw.
- [ ] 📱 Create `docs/tuning.md`: for each gain in `auv/controller.py`, write what happens when you raise or lower it (try it first, then write what you saw).
- [ ] 📱 Add a one-line comment above every gain in `Gains` saying its units.
- [ ] 📱 Open 5 GitHub Issues from the roadmap in `README.md` and label them `enhancement`.

## Week 2 - small code improvements

- [ ] 💻 Add `--no-noise` and `--seed N` flags to `run_demo.py`; note the effect on metrics in the devlog.
- [ ] 💻 Save each run's log to `results/*.csv` and add `results/*.csv` to `.gitignore`.
- [ ] 💻 Fix the known wart: add a depth reference rate limiter in `auv/guidance.py` so the depth reference does not jump at waypoint switches. Add a test that checks the largest per-step jump.
- [ ] 💻 Add a reference filter (first-order or rate limit) for heading steps in `controller.py`; compare overshoot before and after.
- [ ] 💻 Add `disabled` thrusters to `ThrusterConfig.allocate` (mask + re-solve). Test: with one horizontal thruster off, surge and yaw can still be produced.
- [ ] 💻 Test: roll/pitch recover after a 1 N·m impulse.
- [ ] 💻 Test: `wrap()` at the +/-pi boundary and `euler_rate_matrix` refuses or flags pitch near +/-90 deg.

## Week 3 - experiments that produce plots

- [ ] 💻 Current-step scenario: current jumps from 0 to 0.3 m/s at t = 20 s; report recovery time.
- [ ] 💻 Saturation scenario: demand a large depth step and plot thruster saturation plus the allocator scaling.
- [ ] 💻 Monte Carlo: 20 seeds, mean and standard deviation of cross-track RMS in a table in the README.
- [ ] 💻 Add a 3-D trajectory plot (`matplotlib` 3-D axes) with depth inverted.
- [ ] 💻 Make an animated GIF of the mission (`matplotlib.animation`, `pillow`) and embed it in the README.
- [ ] 💻 Lawnmower survey path generator in `auv/guidance.py` plus a scenario that runs it.

## Week 4 - toward real hardware

- [ ] 💻 Complementary filter for roll/pitch from simulated gyro and accelerometer; compare to the raw noisy measurement.
- [ ] 💻 Scalar Kalman filter for depth and heave velocity from a noisy pressure reading.
- [ ] 💻 Remove x/y position from the controller's inputs and run the depth/heading/attitude scenarios with estimates only.
- [ ] 📱 Create `docs/hardware_interface.md`: define the message format (ASCII or binary) between the control computer and the microcontroller: sensor packet in, thrust packet out, rates, checksums.
- [ ] 📱 Create `docs/safety.md`: pool-test checklist (tether, kill switch, leak detector, failsafe on lost comms, buoyancy check).
- [ ] 📱 Create `docs/bom.md`: three build tiers (minimum, mid, higher) with what each lets you test; add prices once you have quotes.
- [ ] 💻 Fixed-rate loop test: run the controller at 50 Hz with timing jitter injected; confirm performance holds.

## Habit

- [ ] 📱 Every day: add a dated entry to `docs/devlog.md` (what you tried, what surprised you). Even on days with no code, this is a real commit.
