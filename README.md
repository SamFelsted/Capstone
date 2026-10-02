# Meltybrain simulator and reference firmware

This repository couples a deterministic planar robot simulation to the same
C++17 controller and safety runtime used by the reference embedded builds. The
desktop UI keeps physical robot truth separate from the firmware's assumptions,
making sensor placement, calibration error, phase drift, traction, saturation,
and arming behavior visible rather than hiding them behind ideal inputs.

## Build and run

Python 3.10+, a C++17 compiler, and CMake 3.20+ are required. From the repository
root:

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
cmake -S . -B build -DPython_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build -j
ctest --test-dir build --output-on-failure
PYTHONPATH=build/python .venv/bin/python -m pytest tests/test_ui.py
PYTHONPATH=build/python .venv/bin/python -m sim.ui.main
```

After building, `./run_sim.sh` launches the same UI using this repository's
virtual environment and native module even when invoked from another directory.

For an offscreen launch/render/lifecycle check:

```sh
QT_QPA_PLATFORM=offscreen PYTHONPATH=build/python .venv/bin/python -m sim.ui.main --smoke
```

That writes `/tmp/meltybrain-ui.png` and shuts the worker down cleanly. The CLI
simulator is `build/sim_cli`.

The Controls tab changes live spin, translation direction and strength, and the
arm request. Configuration values are structural: **Apply & Reset** validates a
complete candidate, then replaces the running simulator only if every value is
valid. Physical sensor geometry and firmware sensor assumptions are intentionally
separate. Reference presets live in `sim/config/robot.json` and
`sim/config/scenario.json`; each includes the schema version, full configuration,
seed, and initial state.

Zero Phase is available only while disarmed. The worker holds its RC command for
receiver latency plus a complete firmware control period in simulated time, so it
also works while paused and does not depend on UI or render timing.

`scenario.json` is a calibrated single-speed reference demo: load it, arm at zero
spin, then command 25% spin and a +X translation demand. Its 2.15 rad translation
phase offset was tuned for the reference model near 112 rad/s. It illustrates the
calibration workflow; changing speed, latency, geometry, or dynamics can change the
required offset and does not guarantee the same world-frame heading.

The Telemetry tab can record CSV samples plus a neighboring JSON metadata file
using the `meltybrain-simulator-recording` v1 schema shared with the CLI. It
contains the epoch's initial state, full configuration, seed, exact worker-accepted
command events, duration, and final state. Starting midway through an epoch writes
the buffered history from time zero; resetting or applying a valid configuration
finalizes that epoch first. Plot a recording with:

```sh
.venv/bin/python tools/plot.py run.csv --output run.png
```

## Production-source relationship

`melty_firmware` compiles `firmware/src/controller.cpp` and
`firmware/src/runtime.cpp`, the production controller/runtime sources used by the
simulator and intended by the board adapters. Simulation truth never enters those
classes; it reaches them through the `Hal` interface. PlatformIO reference
environments build the embedded adapter around that same source.

The checked-in ESP32 and Teensy profiles are unselected reference assumptions,
not robot-ready wiring or calibration. No hardware has been selected or validated.
Before operating a physical spinner, validate electrical levels, receiver and ESC
behavior, sensor range and mounting, timing, mechanical containment, vibration,
failsafe behavior, and an external kill path.

## Current model limits

The simulator is planar and uses simplified rigid-body, tire, motor, battery, and
sensor models. It does not model structural flex, impacts, thermal limits, detailed
battery sag, radio interference, ESC startup protocols, or hardware scheduling
jitter. An accelerometer provides radial spin magnitude rather than absolute world
heading, so phase depends on its reset reference and integration accuracy. Recorded
runs are reproducible for the same build, preset, seed, and command events; they are
not evidence that the physical robot is safe or correctly calibrated.

Desktop simulation time preserves fractional physics ticks independently of the
render rate. Each worker event processes at most 200 ticks and retains at most one
second of wall-clock backlog. Under sustained overload the simulation therefore
slows relative to real time so controls and shutdown remain responsive.
