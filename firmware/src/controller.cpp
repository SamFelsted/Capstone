#include "melty/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace melty {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double wrap_phase(double phase) {
  phase = std::fmod(phase + kPi, kTwoPi);
  if (phase < 0.0) {
    phase += kTwoPi;
  }
  return phase - kPi;
}

bool finite(double value) { return std::isfinite(value); }

}  // namespace

Controller::Controller(const ControllerConfig& config) : config_(config) {}

bool Controller::valid_config(const ControllerConfig& c) {
  return finite(c.sensor_radius_m) && c.sensor_radius_m >= 1.0e-6 &&
         finite(c.sensor_angle_rad) &&
         (c.spin_direction == 1 || c.spin_direction == -1) &&
         finite(c.minimum_phase_spin_rad_s) && c.minimum_phase_spin_rad_s >= 0.0 &&
         finite(c.radial_accel_floor_mps2) && c.radial_accel_floor_mps2 >= 0.0 &&
         finite(c.radial_accel_filter_hz) && c.radial_accel_filter_hz > 0.0 &&
         finite(c.maximum_spin_rad_s) && c.maximum_spin_rad_s > 0.0 &&
         finite(c.spin_kp) && c.spin_kp >= 0.0 && finite(c.spin_ki) &&
         c.spin_ki >= 0.0 && finite(c.spin_integrator_limit) &&
         c.spin_integrator_limit >= 0.0 && finite(c.command_headroom) &&
         c.command_headroom >= 0.0 && c.command_headroom < 1.0 &&
         finite(c.translation_gain) && c.translation_gain >= 0.0 &&
         finite(c.translation_phase_offset_rad);
}

void Controller::reset(Micros now_us, double phase_rad) {
  telemetry_ = {};
  numeric_valid_ = valid_config(config_) && finite(phase_rad);
  telemetry_.phase_rad = numeric_valid_ ? wrap_phase(phase_rad) : 0.0;
  last_update_us_ = now_us;
  initialized_ = true;
}

MotorOutput Controller::fail_numeric() {
  const double safe_phase =
      finite(telemetry_.phase_rad) ? telemetry_.phase_rad : 0.0;
  telemetry_ = {};
  telemetry_.phase_rad = safe_phase;
  numeric_valid_ = false;
  return {};
}

void Controller::disable_output() {
  telemetry_.spin_error_rad_s = 0.0;
  telemetry_.spin_integrator = 0.0;
  telemetry_.common_command = 0.0;
  telemetry_.modulation_command = 0.0;
}

MotorOutput Controller::update(Micros now_us,
                               const AccelerationSample& acceleration,
                               const RcCommand& command,
                               bool output_enabled) {
  if (!initialized_) {
    reset(now_us);
  }
  numeric_valid_ = true;
  if (!valid_config(config_) || !finite(acceleration.x_mps2) ||
      !finite(acceleration.y_mps2) || !finite(command.spin) ||
      command.spin < 0.0 || command.spin > 1.0 ||
      !finite(command.translate_x) || command.translate_x < -1.0 ||
      command.translate_x > 1.0 || !finite(command.translate_y) ||
      command.translate_y < -1.0 || command.translate_y > 1.0) {
    return fail_numeric();
  }

  double dt_s = 0.0;
  if (now_us >= last_update_us_) {
    dt_s = static_cast<double>(now_us - last_update_us_) * 1.0e-6;
  }
  last_update_us_ = now_us;
  // Runtime guards long deadlines. This local bound also avoids a numerical
  // leap if Controller is used directly.
  dt_s = clamp(dt_s, 0.0, 0.1);

  // Sensor is located on body +X. sensor_angle is the CCW rotation from body
  // frame to sensor frame, so this maps sensor acceleration back onto body +X.
  // Centripetal acceleration points inward (-body X).
  const double cosine = std::cos(config_.sensor_angle_rad);
  const double sine = std::sin(config_.sensor_angle_rad);
  const double body_x_acceleration =
      cosine * acceleration.x_mps2 - sine * acceleration.y_mps2;
  if (!finite(body_x_acceleration)) {
    return fail_numeric();
  }
  const double measured_radial = std::max(0.0, -body_x_acceleration);
  if (dt_s > 0.0) {
    const double alpha = 1.0 - std::exp(-2.0 * kPi *
                                        config_.radial_accel_filter_hz * dt_s);
    telemetry_.radial_accel_mps2 +=
        alpha * (measured_radial - telemetry_.radial_accel_mps2);
  } else if (telemetry_.radial_accel_mps2 == 0.0) {
    telemetry_.radial_accel_mps2 = measured_radial;
  }
  if (!finite(telemetry_.radial_accel_mps2)) {
    return fail_numeric();
  }

  const double omega_magnitude = std::sqrt(std::max(
      0.0, telemetry_.radial_accel_mps2 / config_.sensor_radius_m));
  if (!finite(omega_magnitude)) {
    return fail_numeric();
  }
  telemetry_.spin_rad_s =
      static_cast<double>(config_.spin_direction) * omega_magnitude;
  telemetry_.phase_valid =
      telemetry_.radial_accel_mps2 >= config_.radial_accel_floor_mps2 &&
      omega_magnitude >= config_.minimum_phase_spin_rad_s;
  const double phase_candidate =
      telemetry_.phase_rad + telemetry_.spin_rad_s * dt_s;
  if (!finite(phase_candidate)) {
    return fail_numeric();
  }
  telemetry_.phase_rad = wrap_phase(phase_candidate);

  if (!output_enabled) {
    disable_output();
    return {};
  }

  const double target_spin = clamp(command.spin, 0.0, 1.0) *
                             config_.maximum_spin_rad_s;
  telemetry_.spin_error_rad_s = target_spin - omega_magnitude;
  const double maximum_common = 1.0 - config_.command_headroom;
  const double proportional = config_.spin_kp * telemetry_.spin_error_rad_s;
  const double integral_unclamped =
      telemetry_.spin_integrator +
      config_.spin_ki * telemetry_.spin_error_rad_s * dt_s;
  if (!finite(telemetry_.spin_error_rad_s) || !finite(proportional) ||
      !finite(integral_unclamped)) {
    return fail_numeric();
  }
  const double integral_candidate =
      clamp(integral_unclamped, -config_.spin_integrator_limit,
            config_.spin_integrator_limit);
  const double unsaturated_candidate = proportional + integral_candidate;
  if (!finite(unsaturated_candidate)) {
    return fail_numeric();
  }
  const bool saturated_high = unsaturated_candidate > maximum_common;
  const bool saturated_low = unsaturated_candidate < 0.0;
  // Integrate when unsaturated, or when the error would pull the controller
  // back out of its current saturation.
  if ((!saturated_high && !saturated_low) ||
      (saturated_high && telemetry_.spin_error_rad_s < 0.0) ||
      (saturated_low && telemetry_.spin_error_rad_s > 0.0)) {
    telemetry_.spin_integrator = integral_candidate;
  }

  const double common_unclamped = proportional + telemetry_.spin_integrator;
  if (!finite(common_unclamped)) {
    return fail_numeric();
  }
  telemetry_.common_command =
      clamp(common_unclamped, 0.0, maximum_common);

  const double translation_magnitude =
      std::min(1.0, std::hypot(command.translate_x, command.translate_y));
  double modulation = 0.0;
  if (telemetry_.phase_valid && translation_magnitude > 0.0) {
    const double desired_angle =
        std::atan2(command.translate_y, command.translate_x);
    const double wheel_a_force_angle =
        telemetry_.phase_rad +
        static_cast<double>(config_.spin_direction) * 0.5 * kPi +
        config_.translation_phase_offset_rad;
    const double requested = config_.translation_gain * translation_magnitude *
                             std::cos(wheel_a_force_angle - desired_angle);
    if (!finite(requested)) {
      return fail_numeric();
    }
    const double available = std::min(telemetry_.common_command,
                                      1.0 - telemetry_.common_command);
    modulation = clamp(requested, -available, available);
  }
  telemetry_.modulation_command = modulation;

  const MotorOutput output{
      clamp(telemetry_.common_command + modulation, 0.0, 1.0),
      clamp(telemetry_.common_command - modulation, 0.0, 1.0)};
  if (!finite(output.wheel_a) || !finite(output.wheel_b)) {
    return fail_numeric();
  }
  return output;
}

}  // namespace melty
