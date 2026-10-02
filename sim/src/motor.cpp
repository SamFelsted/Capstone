#include "sim/motor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace melty::sim {

Motor::Motor(const MotorParameters& parameters) : parameters_(parameters) {
  const bool valid =
      std::isfinite(parameters.torque_constant_nm_per_a) &&
      parameters.torque_constant_nm_per_a > 0.0 &&
      std::isfinite(parameters.back_emf_v_per_rad_s) &&
      parameters.back_emf_v_per_rad_s > 0.0 &&
      std::isfinite(parameters.resistance_ohm) &&
      parameters.resistance_ohm > 0.0 &&
      std::isfinite(parameters.gear_ratio) && parameters.gear_ratio > 0.0 &&
      std::isfinite(parameters.drivetrain_efficiency) &&
      parameters.drivetrain_efficiency > 0.0 &&
      parameters.drivetrain_efficiency <= 1.0 &&
      std::isfinite(parameters.rotor_inertia_kg_m2) &&
      parameters.rotor_inertia_kg_m2 > 0.0 &&
      std::isfinite(parameters.current_limit_a) &&
      parameters.current_limit_a > 0.0 &&
      std::isfinite(parameters.esc_time_constant_s) &&
      parameters.esc_time_constant_s > 0.0 &&
      std::isfinite(parameters.battery_voltage) &&
      parameters.battery_voltage > 0.0;
  if (!valid) throw std::invalid_argument("invalid motor parameters");
}

void Motor::reset(double wheel_speed_rad_s) {
  if (!std::isfinite(wheel_speed_rad_s)) {
    throw std::invalid_argument("initial wheel speed must be finite");
  }
  const double motor_speed = wheel_speed_rad_s * parameters_.gear_ratio;
  if (!std::isfinite(motor_speed)) {
    throw std::overflow_error("initial motor speed exceeds numerical range");
  }
  applied_voltage_v_ = 0.0;
  motor_speed_rad_s_ = motor_speed;
  current_a_ = 0.0;
}

MotorTelemetry Motor::step(double command, double contact_load_torque_nm,
                           double dt_s) {
  if (!std::isfinite(command) || !std::isfinite(contact_load_torque_nm) ||
      !std::isfinite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument("motor step inputs must be finite and dt positive");
  }
  command = std::clamp(command, 0.0, 1.0);
  const double requested_voltage = command * parameters_.battery_voltage;
  const double response = -std::expm1(-dt_s / parameters_.esc_time_constant_s);
  const double next_voltage =
      applied_voltage_v_ + response * (requested_voltage - applied_voltage_v_);

  // A forward-only ESC neither commands reverse current nor actively brakes.
  const double unconstrained_current =
      (next_voltage -
       parameters_.back_emf_v_per_rad_s * motor_speed_rad_s_) /
      parameters_.resistance_ohm;
  const double next_current =
      std::clamp(unconstrained_current, 0.0, parameters_.current_limit_a);

  const double wheel_drive_torque =
      parameters_.torque_constant_nm_per_a * next_current *
      parameters_.gear_ratio * parameters_.drivetrain_efficiency;
  const double wheel_inertia = parameters_.rotor_inertia_kg_m2 *
                               parameters_.gear_ratio * parameters_.gear_ratio;
  const double wheel_acceleration =
      (wheel_drive_torque - contact_load_torque_nm) / wheel_inertia;
  const double next_motor_speed =
      motor_speed_rad_s_ +
      wheel_acceleration * dt_s * parameters_.gear_ratio;
  if (!std::isfinite(next_voltage) || !std::isfinite(next_current) ||
      !std::isfinite(wheel_drive_torque) ||
      !std::isfinite(wheel_inertia) || wheel_inertia <= 0.0 ||
      !std::isfinite(wheel_acceleration) ||
      !std::isfinite(next_motor_speed)) {
    throw std::overflow_error("motor state became non-finite");
  }
  applied_voltage_v_ = next_voltage;
  current_a_ = next_current;
  motor_speed_rad_s_ = next_motor_speed;

  return {current_a_, wheel_speed_rad_s(), wheel_drive_torque};
}

double Motor::wheel_speed_rad_s() const {
  return motor_speed_rad_s_ / parameters_.gear_ratio;
}

double Motor::current_a() const { return current_a_; }

}  // namespace melty::sim
