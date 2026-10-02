#pragma once

#include <cstdint>

namespace melty {

using Micros = std::uint64_t;

struct AccelerationSample {
  // Sensor-frame kinematic-equivalent acceleration. Board accelerometer HALs
  // negate raw specific force. The configured sensor angle maps this frame to
  // the robot body frame. Timestamp is acquisition time on HAL's monotonic clock.
  double x_mps2{0.0};
  double y_mps2{0.0};
  bool saturated{false};
  Micros timestamp_us{0};
};

struct RcCommand {
  // spin is unipolar [0, 1]. translate_x/y are a world-frame demand in [-1, 1].
  // timestamp_us is when the last complete, valid receiver frame was acquired.
  double spin{0.0};
  double translate_x{0.0};
  double translate_y{0.0};
  bool arm{false};
  bool reset_phase{false};
  Micros timestamp_us{0};
  // A separate hardware gate. It must never be synthesized from the RC arm
  // channel: runtime requires both, and only a real RC arm-low frame can reset
  // the arm interlock.
  bool hardware_enabled{true};
};

struct MotorOutput {
  // Normalized forward ESC demand. This reference design does not request
  // reverse or braking; adapters map zero to their configured safe pulse.
  double wheel_a{0.0};
  double wheel_b{0.0};
};

enum class Fault : std::uint32_t {
  none = 0,
  invalid_configuration = 1u << 0,
  invalid_acceleration = 1u << 1,
  stale_acceleration = 1u << 2,
  invalid_rc = 1u << 3,
  stale_rc = 1u << 4,
  control_deadline = 1u << 5,
  phase_invalid = 1u << 6,
  hal_error = 1u << 7,
  acceleration_saturated = 1u << 8,
  controller_numeric = 1u << 9,
};

constexpr Fault operator|(Fault lhs, Fault rhs) {
  return static_cast<Fault>(static_cast<std::uint32_t>(lhs) |
                            static_cast<std::uint32_t>(rhs));
}

constexpr Fault operator&(Fault lhs, Fault rhs) {
  return static_cast<Fault>(static_cast<std::uint32_t>(lhs) &
                            static_cast<std::uint32_t>(rhs));
}

inline Fault& operator|=(Fault& lhs, Fault rhs) {
  lhs = lhs | rhs;
  return lhs;
}

constexpr bool any(Fault faults) {
  return static_cast<std::uint32_t>(faults) != 0;
}

struct ControllerTelemetry {
  double phase_rad{0.0};
  double spin_rad_s{0.0};
  double radial_accel_mps2{0.0};
  double spin_error_rad_s{0.0};
  double spin_integrator{0.0};
  double common_command{0.0};
  double modulation_command{0.0};
  bool phase_valid{false};
};

struct RuntimeStatus {
  Micros now_us{0};
  Micros last_tick_us{0};
  Micros last_acceleration_us{0};
  Micros last_rc_us{0};
  Fault faults{Fault::none};
  bool armed{false};
  bool arm_interlock_satisfied{false};
  MotorOutput output{};
  ControllerTelemetry controller{};
};

}  // namespace melty
