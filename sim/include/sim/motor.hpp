#pragma once

namespace melty::sim {

struct MotorParameters {
  double torque_constant_nm_per_a{0.0};
  double back_emf_v_per_rad_s{0.0};
  double resistance_ohm{0.0};
  double gear_ratio{0.0};
  double drivetrain_efficiency{0.0};
  double rotor_inertia_kg_m2{0.0};
  double current_limit_a{0.0};
  double esc_time_constant_s{0.0};
  double battery_voltage{0.0};
};

struct MotorTelemetry {
  double current_a{0.0};
  double wheel_speed_rad_s{0.0};
  double wheel_torque_nm{0.0};
};

// A forward-only voltage-source ESC driving a DC motor and fixed-ratio gearbox.
// Positive contact torque is a load opposing positive wheel rotation.
class Motor {
 public:
  explicit Motor(const MotorParameters& parameters);

  void reset(double wheel_speed_rad_s = 0.0);
  MotorTelemetry step(double command, double contact_load_torque_nm,
                      double dt_s);

  double wheel_speed_rad_s() const;
  double current_a() const;

 private:
  MotorParameters parameters_{};
  double applied_voltage_v_{0.0};
  double motor_speed_rad_s_{0.0};
  double current_a_{0.0};
};

}  // namespace melty::sim
