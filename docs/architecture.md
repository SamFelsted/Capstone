# Simulator architecture

`melty_physics` keeps simulated truth behind `melty::sim::Simulator`. The public
class uses a PIMPL so UI bindings do not depend on the plant, motor, delayed
transport queues, or random generator. `Plant` owns the planar chassis and two
physical `Motor` instances. `SimHal` is the only object visible to the firmware
`Runtime`.

The simulator links the same `Runtime` and controller sources used by embedded
builds. It does not copy controller logic and never places physical heading,
true spin, contact force, or undelayed samples into the HAL. At each integration
step the order is:

1. Integrate motors, contacts, and chassis using the previously applied output.
2. Acquire timestamped sensor and receiver frames at scheduled boundaries.
3. Deliver frames whose simulated latency has elapsed.
4. Call `Runtime::tick` at the configured control boundary.
5. Deliver that output to the plant after configured actuator latency.

This ordering gives every command a causal one-step actuation boundary and keeps
the clock entirely virtual. Pausing a UI cannot age data because no wall clock
is consulted.

`SimulationConfig::physical` and `SimulationConfig::sensor` describe simulated
truth. `SimulationConfig::firmware` describes the controller's assumptions and
safety timing. Keeping those domains separate makes calibration errors, sensor
latency, radio latency, clipping, and stale-data behavior testable without a
truth leak.

The command-line program emits time-series CSV plus a JSON recording sidecar.
The sidecar contains the complete physical, sensor, firmware, timing, and seed
configuration, the reset state, and timestamped command events needed to replay
that run. Its nested configuration keys match the UI preset codec, while its
recording schema additionally carries the event timeline. Interactive preset
serialization remains Python/UI code rather than part of the C++ simulator API.
