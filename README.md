# PID Playground

A single-file, no-install simulator for teaching PID control and CTRE Motion Magic to FRC students.
Open `index.html` in any browser — no server, no build step, no dependencies, no robot.

Built for FRC students learning control theory for the first time.

## What it covers

13 levels across two plants. Each one breaks in a specific, realistic way, and each one can
only be solved with the gain the lesson is about.

**Arm (position control)**

| # | Level | The problem | Teaches |
|---|---|---|---|
| 1 | Lift the arm | plain gravity | kP |
| 2 | Stop the bounce | slamming into position | kD |
| 3 | Kill the sag | parks short forever | steady-state error, kG |
| 4 | Smooth operator | step commands rail the motor | Motion Magic profiling |
| 5 | Noisy encoder | rattly chain run jitters the reading | kD amplifies noise |
| 6 | Sticky gearbox | stiction; small commands do nothing | kS |
| 7 | Brownout battery | >8 V sags to 7 V | keeping current demand low |
| 8 | Game piece | mass 2.2x mid-run, kG now wrong | kI absorbs unknown load |
| 9 | Playing defense | random shoves every 1.5 s | disturbance rejection |
| 10 | Match sequence | stow, score low, score high | hitting waypoints on time |

**Flywheel (velocity control)**

| # | Level | The problem | Teaches |
|---|---|---|---|
| 11 | Spin up | hold 4000 RPM | kP *cannot* hold a speed; kV can |
| 12 | Rapid fire | each shot steals ~350 RPM | recovery time = cycle time |

**Diagnose** — no sliders. You get a badly tuned arm and have to name the fault from the
response curve alone, the way you would reading a plot in Phoenix Tuner. Five faults, random order.

## Sandbox mode

A second mode next to Levels. Instead of fixed scenarios you configure the
mechanism itself - arm or elevator, motor and motor count, gear ratio, mass,
length or drum radius - and tune whatever you built.

Type a name or number into the assignment box and it deterministically derives
a mechanism from it, so every student gets a different one and the same code
always reproduces the same machine. Their gains are theirs alone: `kG` is
`12 * m * g * L / (stall * N * ratio * eff)`, so it moves with every parameter.

Sandbox runs in real SI units on a standard brushed-DC torque curve
(`tau = tau_stall * (V/12 - omega_motor/omega_free)`), not the tuned arbitrary
units the levels use. Slider ranges rescale with the mechanism, since an
elevator in metres needs gains an order of magnitude larger than an arm in
radians.

With every gain at zero the motor gets exactly 0 V and the mechanism simply
falls - the arm asymptotes to hanging, the elevator descends at the terminal
velocity where back-EMF braking balances gravity. `verify_sandbox.js` asserts
this across 40 randomly generated mechanisms.

## Physics

The arm is a pendulum with gravity `3.0*cos(theta)`, viscous friction, and a 12 V limit.
The target is 60 degrees, deliberately **not** 90 — at vertical, gravity vanishes exactly at the
setpoint and the whole steady-state-error lesson disappears with it.

The flywheel is a first-order lag, `rpm' = (K*V - rpm)/tau`, with `K = 415 RPM/V` and
`tau = 0.35 s`. Holding 4000 RPM needs 9.64 V, so the ideal `kV` is 2.41 V per 1000 RPM.

Gains map onto Phoenix 6 names, so the instincts transfer straight to a Talon FX.

## Console commands

Open DevTools and type:

    pid.tour()                     play all levels, already solved
    pid.unlockAll()                unlock everything
    pid.level(7)                   jump to a level
    pid.solve()                    auto-tune the current level
    pid.set({P:16,D:2.5,G:3})      set gains directly
    pid.link()                     shareable URL for the current tuning
    pid.resetProgress()            clear saved progress
    pid.help()                     list all of the above

Progress is saved to `localStorage`. Tunings can be shared as a URL:
`index.html#L=6&P=16&D=2.5&G=3&S=1.2` loads that level with those gains — useful for a student
sending a mentor exactly what they were seeing.

## Verifying changes

Every level is checked to be winnable, and to be winnable *by the intended gain*. If you change
the physics or the pass thresholds, re-run these:

    node verify_levels.js      # all 13 levels, 12 candidate tunings
    node verify_flywheel.js    # flywheel spin-up and shot recovery
    node verify_sandbox.js     # sandbox physics, incl. zero gains == zero volts

`verify_levels.js` mirrors the simulation exactly and prints which tunings beat each level.
A level with no winners is a bug.
