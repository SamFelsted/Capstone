# Physics model

The simulator is a deterministic planar rigid-body model intended for controller
development. The checked-in dimensions and motor constants are illustrative,
unmeasured reference values. They must not be used to select hardware or predict
impact performance.

## Drivetrain

Each wheel has an independent forward-only voltage-source ESC, DC motor, and
fixed-ratio gearbox. The ESC voltage approaches `command * battery_voltage` with
the configured first-order time constant. Motor current is

```text
I = clamp((V - Ke * motor_speed) / R, 0, current_limit)
```

and wheel drive torque is `Kt * I * gear_ratio * efficiency`. Rotor inertia is
reflected to the wheel as `motor_inertia * gear_ratio^2`. Contact load torque is
`longitudinal_force * wheel_radius` and opposes rotor acceleration. Zero command
does not request reverse torque or active braking. Consequently this model
captures voltage response, back EMF, current limiting, and rotor energy rather
than converting motor command directly into chassis force.

## Tire contact and chassis

Wheel A is at body `(wheel_offset, 0)` and rolls along body `+Y`. Wheel B is at
`(-wheel_offset, 0)` and rolls along `-Y`. The velocity at each contact is

```text
v_contact = body_velocity + angular_velocity cross wheel_position
slip = wheel_speed * wheel_radius - dot(v_contact, rolling_direction)
```

Longitudinal force is tire stiffness times slip. A lateral force opposes the
contact's lateral velocity using the same stiffness. The combined longitudinal
and lateral vector is limited to the Coulomb circle `mu * normal_force`, with
static normal load `mass * gravity / 2` on each wheel. The equal forward forces
therefore cancel in translation and add in yaw; phase-synchronous differences
produce net translation. Linear and angular viscous drag are applied to the
chassis. Integration uses semi-implicit Euler and substeps no larger than 250
microseconds to resolve the tire/rotor coupling.

The model assumes a flat surface, two permanent contacts, fixed normal loading,
rigid geometry, and no compliance outside the linear tire law. It omits weight
transfer, wheel lift, battery sag, motor inductance and thermal effects,
structural modes, collision geometry, and three-dimensional motion. In
particular, a planar simulation cannot predict weapon impacts, tip-over, or
containment loads.

## Sensor model

The accelerometer is positioned on body `+X` at the physical `sensor.radius_m`.
Its axes are rotated by physical `sensor.angle_rad` relative to the body. These
truth values are deliberately separate from the radius and angle assumed by the
firmware controller. The acceleration at its offset includes center
translation, tangential acceleration `alpha * radius`, and centripetal
acceleration `-omega^2 * radius`. The result is rotated into sensor axes, then
bias and deterministic seeded Gaussian noise are added. Each component is
clipped to the configured range and the sample is marked saturated if either raw
component exceeded that range.

Samples retain acquisition timestamps and become visible to firmware only after
the configured sensor latency. Receiver frames are acquired periodically at the
firmware control rate and delivered after command latency. Firmware motor
outputs reach the physical ESC model after actuator latency. Reset reseeds the
noise source, so the same configuration and commands reproduce the same samples.

An off-axis accelerometer observes radial acceleration magnitude and spin speed,
but it cannot observe absolute world phase during steady rotation. Physical
heading and estimator phase reference are independent reset-state values for
this reason. Translation accuracy depends on their relationship and on spin
estimation error. Motor, tire, transport, and filtering lag also rotate the
average translation vector; `translation_phase_offset_rad` is an explicit
firmware calibration for those effects. It is not derived from simulator truth.

## Time and numerical limits

The simulator clock is integer microseconds and moves only through `advance_*`.
Physics is integrated before samples are delivered; firmware then runs at its
configured control boundary and its output enters the actuator-delay queue.
Sensor and control periods must be integer multiples of the physics tick.
Oversized advances and timestamp overflow are rejected instead of iterating or
wrapping indefinitely.

The numerical domain is intentionally finite. Reset positions are limited to
`±1e6 m`, linear speeds to `±1e3 m/s`, spin to `±1e4 rad/s`, and physical and
estimated angles to `±1e6 rad`. Sensor radius is limited to 10 m, range and bias
to `1e9 m/s²`, and noise deviation to `1e8 m/s²`. Physical parameters likewise
use broad finite engineering bounds; values outside them are unsupported rather
than silently clamped.

For every accepted configuration, the same shared calculation used by the
integrator must resolve rotor/tire, two-contact translation, yaw contact, ESC,
electromechanical, and viscous-drag time scales in at most 100 substeps per
physics tick. Non-finite derived values fail immediately. The combined sensor,
receiver, and actuator transport queues are limited to 100,000 worst-case
in-flight events, computed from latency and acquisition period during
validation. These limits prevent tiny positive parameters or long-latency,
high-rate configurations from turning one nominal tick into an unbounded CPU or
memory request.
