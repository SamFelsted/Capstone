# Verification

Verified on macOS with Apple Clang 17, Python 3.13, and Qt 6.11.

## Checks

- CMake desktop build, including the Python extension: passed.
- CTest firmware phase, runtime, plant, and closed-loop suites: 4/4 passed.
- Offscreen UI, preset, lifecycle, configuration-isolation, and recording-replay tests: 13/13 passed.
- UndefinedBehaviorSanitizer and float-cast-overflow checks: 4/4 native suites passed.
- PlatformIO `esp32dev` and `teensy41` reference builds: passed with C++17.
- Desktop launcher smoke test from outside the repository: passed.
- CLI recording replay reproduced position and spin within serialized floating-point precision.

Run the standard checks after building as described in the root README:

```sh
ctest --test-dir build --output-on-failure
QT_QPA_PLATFORM=offscreen PYTHONPATH=build/python .venv/bin/python -m pytest tests/test_ui.py
QT_QPA_PLATFORM=offscreen ./run_sim.sh --smoke
.venv/bin/pio run -d firmware
```

For undefined-behavior checks:

```sh
cmake -S . -B build-ubsan -DMELTY_BUILD_PYTHON=OFF \
  -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined,float-cast-overflow -fno-omit-frame-pointer'
cmake --build build-ubsan -j
ctest --test-dir build-ubsan --output-on-failure
```

## Limits

AddressSanitizer could not run on this machine: its Apple runtime deadlocked
during initialization before `main`. An empty program reproduced the same failure;
this check is unavailable, not passed.

No firmware was flashed and no physical robot was tested. Board wiring,
calibration, ESC behavior, execution timing, and an independent motor-disable
mechanism still require hardware validation. The planar model and its reference
motor parameters are illustrative; they do not validate impacts or three-dimensional
robot motion.
