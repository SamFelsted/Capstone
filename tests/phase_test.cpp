#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "melty/controller.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

double angular_error(double a, double b) {
  return std::remainder(a - b, 2.0 * kPi);
}

}  // namespace

int main() {
  melty::ControllerConfig config{};
  config.spin_kp = 0.005;
  config.spin_ki = 0.02;
  config.maximum_spin_rad_s = 200.0;
  config.translation_gain = 0.4;
  melty::Controller controller(config);
  controller.reset(0, 0.3);

  melty::AccelerationSample acceleration{};
  acceleration.x_mps2 = -config.sensor_radius_m * 100.0 * 100.0;
  melty::RcCommand command{};
  command.spin = 0.6;  // target 120 rad/s while measured speed is 100.

  for (melty::Micros time = 1000; time <= 500000; time += 1000) {
    acceleration.timestamp_us = time;
    command.timestamp_us = time;
    (void)controller.update(time, acceleration, command);
  }
  require(controller.telemetry().phase_valid,
          "high radial acceleration should produce valid phase");
  require(std::abs(controller.telemetry().spin_rad_s - 100.0) < 0.1,
          "spin estimate should converge to sqrt(a/r)");

  const double phase_before = controller.telemetry().phase_rad;
  for (melty::Micros time = 501000; time <= 600000; time += 1000) {
    acceleration.timestamp_us = time;
    command.timestamp_us = time;
    (void)controller.update(time, acceleration, command);
  }
  const double expected = phase_before + 100.0 * 0.1;
  require(std::abs(angular_error(controller.telemetry().phase_rad, expected)) < 0.01,
          "phase should integrate estimated spin using deterministic timestamps");

  const double wheel_a_tangent =
      controller.telemetry().phase_rad + 0.5 * kPi;
  command.translate_x = std::cos(wheel_a_tangent);
  command.translate_y = std::sin(wheel_a_tangent);
  acceleration.timestamp_us = 601000;
  command.timestamp_us = 601000;
  const melty::MotorOutput modulated =
      controller.update(601000, acceleration, command);
  require(modulated.wheel_a > modulated.wheel_b,
          "world demand along wheel-A tangent should increase wheel A");
  require(modulated.wheel_a <= 1.0 && modulated.wheel_b >= 0.0,
          "modulation must preserve normalized output bounds");

  melty::ControllerConfig saturation_config{};
  saturation_config.spin_integrator_limit = 0.5;
  melty::Controller saturated(saturation_config);
  saturated.reset(0);
  melty::AccelerationSample stopped{};
  melty::RcCommand maximum{};
  maximum.spin = 1.0;
  for (melty::Micros time = 1000; time <= 100000; time += 1000) {
    stopped.timestamp_us = time;
    maximum.timestamp_us = time;
    (void)saturated.update(time, stopped, maximum);
  }
  require(std::abs(saturated.telemetry().spin_integrator) < 1.0e-12,
          "conditional anti-windup should not accumulate into high saturation");
  require(!saturated.telemetry().phase_valid,
          "stationary acceleration cannot provide valid phase");
  require(saturated.telemetry().common_command > 0.0,
          "phase-invalid startup must retain common spin effort");

  melty::ControllerConfig rotated_config = config;
  rotated_config.sensor_angle_rad = 0.5 * kPi;
  melty::Controller rotated_sensor(rotated_config);
  rotated_sensor.reset(0);
  melty::AccelerationSample rotated_acceleration{};
  // Inward body-X acceleration expressed in a sensor frame rotated +90 deg.
  rotated_acceleration.y_mps2 = 250.0;
  (void)rotated_sensor.update(0, rotated_acceleration, command);
  require(std::abs(rotated_sensor.telemetry().spin_rad_s - 100.0) < 0.1,
          "sensor angle must rotate the radial projection into the body frame");

  melty::Controller tangential_only(config);
  tangential_only.reset(0);
  melty::AccelerationSample tangential_acceleration{};
  tangential_acceleration.y_mps2 = 250.0;
  (void)tangential_only.update(0, tangential_acceleration, command);
  require(tangential_only.telemetry().spin_rad_s == 0.0,
          "tangential acceleration must not masquerade as centripetal spin");

  melty::ControllerConfig tiny_radius = config;
  tiny_radius.sensor_radius_m = std::numeric_limits<double>::denorm_min();
  require(!melty::Controller::valid_config(tiny_radius),
          "computationally unsafe denormal sensor radius must be rejected");

  melty::ControllerConfig extreme_config = config;
  extreme_config.sensor_radius_m = 1.0e-6;
  melty::Controller extreme(extreme_config);
  extreme.reset(0);
  melty::AccelerationSample extreme_acceleration{};
  extreme_acceleration.x_mps2 = -std::numeric_limits<double>::max();
  const melty::MotorOutput extreme_output =
      extreme.update(0, extreme_acceleration, maximum);
  require(!extreme.numeric_valid(),
          "finite inputs producing non-finite derived state must fail closed");
  require(extreme_output.wheel_a == 0.0 && extreme_output.wheel_b == 0.0 &&
              std::isfinite(extreme.telemetry().phase_rad) &&
              std::isfinite(extreme.telemetry().spin_rad_s) &&
              !extreme.telemetry().phase_valid,
          "numeric failure must retain finite telemetry and safe output");

  std::cout << "phase_test passed\n";
  return 0;
}
