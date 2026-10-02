#pragma once

#include "melty/types.hpp"

namespace melty {

struct ControllerConfig {
  double sensor_radius_m{0.025};
  double sensor_angle_rad{0.0};
  int spin_direction{1};  // +1 CCW, -1 CW.
  double minimum_phase_spin_rad_s{20.0};
  double radial_accel_floor_mps2{10.0};
  double radial_accel_filter_hz{25.0};
  double maximum_spin_rad_s{450.0};

  double spin_kp{0.012};
  double spin_ki{0.10};
  double spin_integrator_limit{0.45};
  double command_headroom{0.08};
  double translation_gain{0.45};
  // Wheel A's positive-force angle is phase + spin_sign*pi/2 plus this measured
  // mechanical/electrical compensation.
  double translation_phase_offset_rad{0.0};
};

class Controller {
 public:
  explicit Controller(const ControllerConfig& config = {});

  static bool valid_config(const ControllerConfig& config);
  void reset(Micros now_us, double phase_rad = 0.0);
  void disable_output();
  MotorOutput update(Micros now_us, const AccelerationSample& acceleration,
                     const RcCommand& command, bool output_enabled = true);

  const ControllerTelemetry& telemetry() const { return telemetry_; }
  bool numeric_valid() const { return numeric_valid_; }

 private:
  MotorOutput fail_numeric();

  ControllerConfig config_{};
  ControllerTelemetry telemetry_{};
  Micros last_update_us_{0};
  bool initialized_{false};
  bool numeric_valid_{true};
};

}  // namespace melty
