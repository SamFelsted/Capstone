#include "sim/motor.hpp"

#include <algorithm>
#include <cmath>

namespace melty::sim {

Motor::Motor(const MotorParameters& parameters) : parameters_(parameters) {}

void Motor::reset(double wheel_speed_rad_s) {
  applied_voltage_v_ = 0.0;
  motor_speed_rad_s_ = wheel_speed_rad_s * parameters_.gear_ratio;
  current_a_ = 0.0;
}

MotorTelemetry Motor::step(double command, double contact_load_torque_nm,
                           double dt_s) {
  command = std::clamp(command, 0.0, 1.0);
  const double requested_voltage = command * parameters_.battery_voltage;
  const double response = -std::expm1(-dt_s / parameters_.esc_time_constant_s);
  applied_voltage_v_ += response * (requested_voltage - applied_voltage_v_);

  // A forward-only ESC neither commands reverse current nor actively brakes.
  const double unconstrained_current =
      (applied_voltage_v_ -
       parameters_.back_emf_v_per_rad_s * motor_speed_rad_s_) /
      parameters_.resistance_ohm;
  current_a_ =
      std::clamp(unconstrained_current, 0.0, parameters_.current_limit_a);

  const double wheel_drive_torque =
      parameters_.torque_constant_nm_per_a * current_a_ *
      parameters_.gear_ratio * parameters_.drivetrain_efficiency;
  const double wheel_inertia = parameters_.rotor_inertia_kg_m2 *
                               parameters_.gear_ratio * parameters_.gear_ratio;
  const double wheel_acceleration =
      (wheel_drive_torque - contact_load_torque_nm) / wheel_inertia;
  motor_speed_rad_s_ +=
      wheel_acceleration * dt_s * parameters_.gear_ratio;

  return {current_a_, wheel_speed_rad_s(), wheel_drive_torque};
}

double Motor::wheel_speed_rad_s() const {
  return motor_speed_rad_s_ / parameters_.gear_ratio;
}

double Motor::current_a() const { return current_a_; }

}  // namespace melty::sim
