#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "melty/runtime.hpp"

namespace melty::sim {

struct PhysicalConfig {
  double mass_kg{1.36};
  double body_radius_m{0.09};
  double moment_of_inertia_kg_m2{0.0055};
  double wheel_radius_m{0.02};
  double wheel_offset_m{0.065};
  double motor_torque_constant_nm_per_a{0.006};
  double motor_back_emf_v_per_rad_s{0.006};
  double motor_resistance_ohm{0.35};
  double gear_ratio{1.0};
  double drivetrain_efficiency{0.85};
  double motor_inertia_kg_m2{0.000008};
  double motor_current_limit_a{30.0};
  double motor_time_constant_s{0.025};
  double tire_friction_coefficient{0.8};
  double tire_longitudinal_stiffness_n_per_mps{30.0};
  double linear_drag_n_per_mps{0.15};
  double angular_drag_nm_per_rad_s{0.0008};
  double battery_voltage{8.4};
};

struct SensorModelConfig {
  double radius_m{0.025};
  double angle_rad{0.0};
  double max_acceleration_mps2{980.665};
  double noise_stddev_mps2{0.5};
  double bias_x_mps2{0.0};
  double bias_y_mps2{0.0};
  Micros sample_period_us{1'000};
  Micros latency_us{500};
};

struct SimulationConfig {
  PhysicalConfig physical{};
  SensorModelConfig sensor{};
  RuntimeConfig firmware{};
  Micros physics_tick_us{250};
  Micros command_latency_us{2'000};
  Micros actuator_latency_us{1'000};
  std::uint64_t scenario_seed{1};
};

struct ResetState {
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double spin_rad_s{0.0};
  // Estimator reference is deliberately independent of physical heading so
  // phase offset and calibration error can be simulated.
  double estimated_phase_rad{0.0};
};

struct UserCommand {
  double spin{0.0};
  double translate_x{0.0};
  double translate_y{0.0};
  bool arm{false};
  bool reset_phase{false};
};

struct WheelTelemetry {
  double command{0.0};
  double motor_current_a{0.0};
  double wheel_speed_rad_s{0.0};
  double slip_mps{0.0};
  double requested_force_n{0.0};
  double applied_force_n{0.0};
  double normal_force_n{0.0};
  bool traction_limited{false};
};

struct Snapshot {
  Micros time_us{0};
  double x_m{0.0};
  double y_m{0.0};
  double heading_rad{0.0};
  double vx_mps{0.0};
  double vy_mps{0.0};
  double spin_rad_s{0.0};
  AccelerationSample sensed_acceleration{};
  RuntimeStatus firmware{};
  WheelTelemetry wheel_a{};
  WheelTelemetry wheel_b{};
  // World-frame wheel drive force (longitudinal only) low-passed with a
  // 0.15 s time constant: the net push averaged over revolutions.
  double mean_drive_force_x_n{0.0};
  double mean_drive_force_y_n{0.0};
  // Inputs firmware consumed on its most recent control tick.
  AccelerationSample consumed_acceleration{};
  RcCommand consumed_command{};
  // Time and fault mask of the most recent armed→disarmed transition, latched
  // because faults such as saturation can clear on the very next tick.
  Micros last_disarm_us{0};
  Fault last_disarm_faults{Fault::none};
};

class Simulator {
 public:
  explicit Simulator(const SimulationConfig& config = {});
  ~Simulator();
  Simulator(Simulator&&) noexcept;
  Simulator& operator=(Simulator&&) noexcept;
  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

  static std::vector<std::string> validate(const SimulationConfig& config);
  const SimulationConfig& config() const;
  void reset(const ResetState& state = {});
  void set_command(const UserCommand& command);
  void advance_ticks(std::size_t ticks);
  void advance_for(Micros duration_us);
  Snapshot snapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace melty::sim
