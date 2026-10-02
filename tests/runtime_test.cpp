#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "melty/runtime.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

class FakeHal final : public melty::Hal {
 public:
  melty::Micros now{0};
  melty::AccelerationSample acceleration{};
  melty::RcCommand command{};
  melty::MotorOutput written{};
  bool acceleration_ok{true};
  bool rc_ok{true};
  unsigned writes{0};

  melty::Micros now_us() override { return now; }
  bool read_acceleration(melty::AccelerationSample& value) override {
    value = acceleration;
    return acceleration_ok;
  }
  bool read_rc(melty::RcCommand& value) override {
    value = command;
    return rc_ok;
  }
  void write_motors(const melty::MotorOutput& value) override {
    written = value;
    ++writes;
  }

  void stamp() {
    acceleration.timestamp_us = now;
    command.timestamp_us = now;
  }
};

bool has(melty::Fault all, melty::Fault flag) {
  return melty::any(all & flag);
}

void tick_at(FakeHal& hal, melty::Runtime& runtime, melty::Micros time) {
  hal.now = time;
  hal.stamp();
  runtime.tick();
}

}  // namespace

int main() {
  FakeHal hal;
  melty::RuntimeConfig config{};
  config.arm_confirm_ticks = 3;
  melty::Runtime runtime(hal, config);
  require(hal.writes == 1 && hal.written.wheel_a == 0.0,
          "construction/reset must apply safe output immediately");

  hal.acceleration.x_mps2 = -250.0;
  hal.command.spin = 0.0;
  hal.command.arm = false;
  tick_at(hal, runtime, 1000);
  require(runtime.status().arm_interlock_satisfied,
          "fresh low-spin disarmed frame should satisfy initial interlock");

  hal.command.spin = 0.5;
  hal.command.arm = true;
  tick_at(hal, runtime, 2000);
  tick_at(hal, runtime, 3000);
  require(!runtime.status().armed,
          "arming must wait for configured distinct receiver frames");
  tick_at(hal, runtime, 4000);
  require(runtime.status().armed, "third fresh arm frame should arm runtime");
  require(hal.written.wheel_a > 0.0 || hal.written.wheel_b > 0.0,
          "armed runtime should apply controller output");

  const double phase_armed = runtime.status().controller.phase_rad;
  hal.command.arm = false;
  hal.command.spin = 0.0;
  tick_at(hal, runtime, 4500);
  require(runtime.status().controller.spin_integrator == 0.0 &&
              runtime.status().controller.common_command == 0.0,
          "disarm must clear stored controller torque state");
  require(runtime.status().controller.phase_rad != phase_armed,
          "healthy disarmed tick must continue estimator phase integration");

  hal.command.arm = true;
  hal.command.spin = 0.5;
  tick_at(hal, runtime, 4600);
  require(!runtime.status().armed,
          "drop-arm event must require a new low-spin disarmed handshake");

  // Repeating a cached receiver frame must not satisfy the arm count.
  runtime.reset();
  hal.acceleration.x_mps2 = 0.0;
  hal.command.arm = false;
  hal.command.spin = 0.0;
  tick_at(hal, runtime, 5000);
  hal.command.arm = true;
  hal.command.spin = 0.5;
  hal.now = 6000;
  hal.stamp();
  runtime.tick();
  hal.now = 7000;
  hal.acceleration.timestamp_us = 7000;
  runtime.tick();
  hal.now = 8000;
  hal.acceleration.timestamp_us = 8000;
  runtime.tick();
  require(!runtime.status().armed,
          "cached armed RC frame must count only once");

  // Complete arming with two new frames, using zero acceleration. Phase is
  // invalid but common effort remains available for spin-up.
  tick_at(hal, runtime, 9000);
  tick_at(hal, runtime, 10000);
  tick_at(hal, runtime, 11000);
  require(runtime.status().armed,
          "invalid phase alone must not clear the arm handshake");
  require(has(runtime.status().faults, melty::Fault::phase_invalid),
          "phase invalidity should remain observable");
  require(hal.written.wheel_a > 0.0 && hal.written.wheel_b > 0.0,
          "spin-up should command common motor effort without valid phase");

  hal.acceleration.saturated = true;
  tick_at(hal, runtime, 12000);
  require(!runtime.status().armed && hal.written.wheel_a == 0.0,
          "accelerometer saturation must disarm in the same tick");
  require(has(runtime.status().faults, melty::Fault::acceleration_saturated),
          "saturation fault should be reported");

  hal.acceleration.saturated = false;
  hal.acceleration.x_mps2 = std::numeric_limits<double>::quiet_NaN();
  tick_at(hal, runtime, 13000);
  require(has(runtime.status().faults, melty::Fault::invalid_acceleration),
          "non-finite acceleration should be rejected");
  require(hal.written.wheel_a == 0.0 && hal.written.wheel_b == 0.0,
          "invalid numeric input must apply safe output");

  hal.acceleration.x_mps2 = 0.0;
  hal.now = 13000 + config.acceleration_timeout_us + 1;
  // Deliberately keep old input timestamps to exercise both deadline and age.
  runtime.tick();
  require(has(runtime.status().faults, melty::Fault::control_deadline),
          "late tick should report deadline fault");
  require(has(runtime.status().faults, melty::Fault::stale_acceleration),
          "old sensor sample should report stale fault");
  require(hal.written.wheel_a == 0.0 && hal.written.wheel_b == 0.0,
          "safety fault must write zero in the same transaction");

  std::cout << "runtime_test passed\n";
  return 0;
}
