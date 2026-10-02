# Meltybrain implementation contract

The project separates a deterministic planar simulator from a production-shaped
firmware library. `melty_firmware` is C++17 and contains no simulator truth or
platform calls. The same controller and runtime sources compile on the host,
ESP32 DevKit, and Teensy 4.1. A `Hal` supplies timestamped acceleration and RC
frames and accepts normalized forward-only motor output.

The reference robot uses two wheels on opposite sides of the body. Both wheels
drive tangentially in the same spin direction. Translation is created by equal
and opposite phase-synchronous modulation around a common spin command. Output
saturation reserves configurable command headroom, and the PI integrator uses
conditional anti-windup.

## Time, phase, and arming

All public timestamps are unsigned integer microseconds on one monotonic epoch.
An input timestamp is acquisition-completion time, not tick-entry or delivery
time. Runtime samples the clock at tick entry to check continuity from the prior
valid acquisition completion, then again after both HAL reads. The post-read
time governs freshness, controller integration, status `now_us`, and the next
`last_tick_us`; read duration therefore counts against the control deadline.
Backward time, a sample later than the post-read clock, one older than its
configured timeout, a non-finite value, or a control tick after the deadline
disarms in that tick. Board HALs must extend Arduino's
wrapping 32-bit `micros()` result to 64 bits. The simulator clock starts at zero
and advances only by `advance_ticks` or `advance_for`.
`RuntimeConfig::control_period_us` is the nominal firmware cadence; the simulator
and board loop schedule runtime ticks at that period. `maximum_tick_interval_us`
is the separate missed-deadline safety limit.
Simulator `command_latency_us` delays RC transport into firmware, while
`actuator_latency_us` independently delays firmware output reaching the plant.

The sensor is modeled at body `+X`, and `sensor_angle_rad` is its counterclockwise
frame rotation from the body frame. The HAL reports kinematic-equivalent
acceleration (real accelerometer adapters negate raw specific force). The
estimator rotates the sample into the body frame and projects only inward radial
acceleration, rejecting tangential acceleration rather than using vector
magnitude. With sensor radius `r`, it uses `sqrt(a_radial/r)` and the configured
spin sign, then integrates phase. An accelerometer does not provide an absolute world
heading during steady rotation; `reset_phase` establishes phase zero. The phase
estimate is invalid below the configured acceleration/spin floors. A gyro is not
required because common parts saturate well below expected meltybrain spin rate.
Simulator reset state carries physical heading and estimated phase separately so
the UI can expose phase-offset and drift scenarios honestly.

Invalid phase during startup does not prevent spin-up: the controller applies
common spin effort and suppresses translation modulation until phase becomes
valid. `phase_invalid` is reported as live status. Invalid or stale samples still
disarm immediately.

Wheel A is at body `+X`, so its positive tangential force direction is estimated
phase plus `spin_direction*pi/2`. Translation modulation is referenced to that
tangent plus `translation_phase_offset_rad`, which accounts for measured mounting,
wiring, and commutation delay. Wheel B receives opposite modulation.

`Runtime::reset` writes safe output immediately. Arming requires at least one
fresh disarmed RC frame with spin at or below `arm_spin_max`, followed by
`arm_confirm_ticks` fresh armed frames. Dropping arm, stale data, bad values, HAL
failure, or a deadline miss clears the handshake and writes zero.
After any such event the disarmed-then-armed sequence is required again.
The physical hardware-enable input is carried separately from the RC arm switch.
Dropping hardware enable clears the handshake, and raising it while RC arm is
still high cannot establish the interlock; the receiver must provide a real
arm-low frame while hardware is enabled before subsequent arm-high frames count.
PWM channels are captured asynchronously, but the HAL advances `RcCommand`'s
timestamp only after every required channel has produced a new complete pulse.
Partial channel updates reuse the previous complete-frame timestamp and cannot
advance the arming confirmation count.
Accelerometer clipping is explicit in `AccelerationSample::saturated`; runtime
reports `acceleration_saturated` and disarms instead of treating the clipped
magnitude as a trustworthy low-speed measurement.
On healthy disarmed ticks the estimator continues integrating phase while the PI
integrator and motor terms are held at zero. This preserves coast-down state
without reusing stale torque when the arm handshake is completed again.
Controller configuration rejects sensor radii below 1 micrometer, and each
derived control stage checks finiteness. A finite input that overflows during
projection, estimation, integration, or mixing produces zero output,
`controller_numeric`, and a cleared arm handshake rather than NaN telemetry.

## Public classes

- `melty::Controller` owns phase/spin estimation, spin PI, anti-windup, and
  phase-based wheel modulation. It consumes only timestamped acceleration and RC
  commands.
- `melty::Runtime` owns input validation, freshness/deadline checks, the arming
  state machine, and same-tick output application through `melty::Hal`.
- `melty::sim::Simulator` owns a deterministic integer-microsecond clock,
  seeded sensor noise, rigid-body/wheel dynamics, a simulated HAL, and the same
  `Runtime` used on hardware.
- `SimulationConfig::physical` describes simulated robot truth. Its values are
  intentionally distinct from `SimulationConfig::firmware.controller`, which
  describes what firmware assumes. This supports calibration-error scenarios.

The UI-facing simulator API is `reset`, `set_command`, `advance_ticks`,
`advance_for`, and `snapshot`. A snapshot contains current physical truth,
measured acceleration, estimated controller state, runtime faults and arming,
motor demands, applied wheel forces, and traction limiting. Preset and JSON
serialization remain outside the C++ simulation contract.

## Hardware reference and limits

`platformio.ini` provides `esp32dev` and `teensy41` Arduino builds around an
H3LIS331DL at I2C address `0x18`, five PWM receiver inputs, and two forward-only
PWM ESC outputs. Hardware output stays disabled unless the dedicated enable pin
is asserted. The firmware image additionally stays inert unless built with
`MELTY_ENABLE_REFERENCE_HARDWARE`; the checked-in PlatformIO environments omit
that define. The checked-in pin assignments, receiver endpoints, accelerometer
scale, axis orientation, ESC pulse range, sensor radius, spin direction, and PI
gains are reference assumptions that must be measured and reviewed for the
actual robot. Missing sensor/receiver data is reported as failure; adapters never
invent samples. There is no reverse or active-brake command.

The ESP32 DevKit reference uses I2C SDA/SCL 21/22, RC spin/X/Y/arm/reset pins
32/33/25/26/27, ESC A/B pins 18/19, and hardware enable pin 23. The Teensy 4.1
reference uses fixed Wire SDA/SCL 18/19, RC pins 2/3/4/5/6, ESC pins 7/8, and
enable pin 9. Both assume 1000/1500/2000 us RC endpoints, forward-only
1000--2000 us ESC input, the H3LIS331DL +/-100 g range at 0.47884 m/s^2 per
12-bit count, a conservative illustrative 150 rad/s maximum command below the
sensor's roughly 198 rad/s clipping point at 25 mm radius, and sensor axes
configured by `sensor_angle_rad`. The physical
enable inputs use internal pulldowns; production wiring needs an external
fail-safe bias and an independent kill path.

This is a simulator and reference firmware architecture, not a claim of a
robot-ready safety system. Hardware selection is unresolved. Before physical
use, validate electrical levels, interrupt latency, failsafe behavior, ESC arming
requirements, mechanical containment, vibration isolation, and a tested external
kill path.
The software deadline check can react only when `tick()` resumes; it cannot
protect against a hung processor. An independent watchdog that directly forces
safe output, along with the external kill path, remains required hardware work.
