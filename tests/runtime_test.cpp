#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "melty/board_profile.hpp"
#include "melty/rc_pwm_input.hpp"
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

void test_pwm_frame_assembly() {
  melty::PwmRcFrameAssembler assembler;
  melty::PwmRcCapture capture{};
  capture.pulse_us = {1000, 1500, 1500, 2000, 1000};
  capture.ended_us = {1000, 1100, 1200, 1300, 1400};
  capture.sequence.fill(1);
  capture.have_pulse.fill(true);
  capture.now_raw_us = 2000;
  capture.now_us = 2000;
  capture.hardware_enabled = true;
  melty::RcCommand command{};
  require(assembler.update(capture, command),
          "first complete set of RC channels should publish");
  const melty::Micros first_timestamp = command.timestamp_us;
  require(first_timestamp == 1000 && command.arm,
          "complete frame timestamp should conservatively use oldest channel");

  // Advance unrelated channels asynchronously. The cached arm sample and
  // complete-frame timestamp must not be republished as fresh.
  capture.now_raw_us = 3500;
  capture.now_us = 3500;
  for (std::size_t channel : {0u, 1u, 2u, 4u}) {
    ++capture.sequence[channel];
    capture.ended_us[channel] = 2500 + static_cast<std::uint32_t>(100 * channel);
    require(assembler.update(capture, command),
            "partial fresh channels should retain the valid cached frame");
    require(command.timestamp_us == first_timestamp,
            "partial channel updates must not advance command timestamp");
  }
  ++capture.sequence[3];
  capture.ended_us[3] = 2800;
  capture.now_raw_us = 3600;
  capture.now_us = 3600;
  require(assembler.update(capture, command) &&
              command.timestamp_us > first_timestamp,
          "timestamp should advance only after every channel advances");
  capture.now_raw_us = 200000;
  capture.now_us = 200000;
  require(!assembler.update(capture, command),
          "any stale constituent channel must invalidate cached RC output");

  // Explicit validity bits allow a pulse ending exactly at 32-bit wrap while
  // still rejecting a boot-time falling edge that had no preceding rise.
  melty::PwmRcFrameAssembler rollover_assembler;
  melty::PwmRcCapture rollover{};
  rollover.pulse_us.fill(1500);
  rollover.ended_us.fill(0);
  rollover.sequence.fill(1);
  rollover.have_pulse.fill(true);
  rollover.now_raw_us = 0;
  rollover.now_us = (melty::Micros{1} << 32);
  rollover.hardware_enabled = true;
  require(rollover_assembler.update(rollover, command) &&
              command.timestamp_us == rollover.now_us,
          "valid capture at micros rollover must not collide with a sentinel");
  melty::PwmRcFrameAssembler missing_edge_assembler;
  rollover.have_pulse[2] = false;
  require(!missing_edge_assembler.update(rollover, command),
          "channel without a complete rise/fall pulse must be rejected");
}

void test_hardware_enable_handshake() {
  FakeHal hal;
  melty::RuntimeConfig config{};
  config.arm_confirm_ticks = 2;
  melty::Runtime runtime(hal, config);
  hal.acceleration.x_mps2 = 0.0;
  hal.command.spin = 0.0;
  hal.command.arm = true;
  hal.command.hardware_enabled = false;
  tick_at(hal, runtime, 1000);
  tick_at(hal, runtime, 2000);
  require(!runtime.status().arm_interlock_satisfied,
          "disabled hardware with RC arm high cannot satisfy interlock");

  hal.command.hardware_enabled = true;
  tick_at(hal, runtime, 3000);
  tick_at(hal, runtime, 4000);
  require(!runtime.status().armed &&
              !runtime.status().arm_interlock_satisfied,
          "raising hardware enable with RC arm high cannot arm");

  hal.command.arm = false;
  tick_at(hal, runtime, 5000);
  require(runtime.status().arm_interlock_satisfied,
          "real RC arm-low frame while enabled should establish interlock");
  hal.command.arm = true;
  tick_at(hal, runtime, 6000);
  tick_at(hal, runtime, 7000);
  require(runtime.status().armed,
          "fresh RC arm-high frames may arm after real low frame");

  runtime.reset();
  hal.command.arm = false;
  hal.command.hardware_enabled = false;
  tick_at(hal, runtime, 8000);
  hal.command.hardware_enabled = true;
  hal.now = 9000;
  hal.acceleration.timestamp_us = 9000;
  runtime.tick();  // Deliberately reuse the low RC frame captured while disabled.
  require(!runtime.status().arm_interlock_satisfied,
          "cached arm-low frame from disabled hardware cannot reset interlock");
  tick_at(hal, runtime, 10000);
  require(runtime.status().arm_interlock_satisfied,
          "new arm-low frame after hardware enable may reset interlock");
}

void test_numeric_fail_closed() {
  FakeHal hal;
  melty::RuntimeConfig config{};
  config.arm_confirm_ticks = 1;
  config.controller.spin_kp = std::numeric_limits<double>::max();
  melty::Runtime runtime(hal, config);
  hal.command.spin = 0.0;
  hal.command.arm = false;
  tick_at(hal, runtime, 1000);
  hal.command.spin = 1.0;
  hal.command.arm = true;
  tick_at(hal, runtime, 2000);
  require(!runtime.status().armed &&
              has(runtime.status().faults, melty::Fault::controller_numeric) &&
              hal.written.wheel_a == 0.0 && hal.written.wheel_b == 0.0,
          "non-finite derived controller state must disarm and write safe output");
}

}  // namespace

int main() {
  test_pwm_frame_assembly();
  test_hardware_enable_handshake();
  test_numeric_fail_closed();

  const melty::BoardProfile esp = melty::esp32_devkit_reference_profile();
  const melty::BoardProfile teensy = melty::teensy41_reference_profile();
  require(melty::valid_board_profile(esp) &&
              melty::valid_board_profile(teensy) &&
              esp.runtime.controller.maximum_spin_rad_s == 150.0 &&
              teensy.runtime.controller.maximum_spin_rad_s == 150.0,
          "checked-in board profiles should be valid and respect sensor range");
  melty::BoardProfile invalid_profile = esp;
  invalid_profile.esc_b_pin = invalid_profile.esc_a_pin;
  require(!melty::valid_board_profile(invalid_profile),
          "board profile must reject duplicate safety-critical pins");

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
