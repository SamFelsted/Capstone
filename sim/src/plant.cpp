#include "sim/plant.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace melty::sim {
namespace {

constexpr double kGravityMps2 = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaximumSubsteps = 100;

bool finite_in_range(double value, double minimum, double maximum) {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

bool supported_reset_state(const ResetState& state) {
  return finite_in_range(state.x_m, -1.0e6, 1.0e6) &&
         finite_in_range(state.y_m, -1.0e6, 1.0e6) &&
         finite_in_range(state.heading_rad, -1.0e6, 1.0e6) &&
         finite_in_range(state.vx_mps, -1.0e3, 1.0e3) &&
         finite_in_range(state.vy_mps, -1.0e3, 1.0e3) &&
         finite_in_range(state.spin_rad_s, -1.0e4, 1.0e4) &&
         finite_in_range(state.estimated_phase_rad, -1.0e6, 1.0e6);
}

bool supported_physical_config(const PhysicalConfig& p) {
  return finite_in_range(p.mass_kg, 1.0e-3, 1.0e4) &&
         finite_in_range(p.body_radius_m, 1.0e-4, 10.0) &&
         finite_in_range(p.moment_of_inertia_kg_m2, 1.0e-9, 1.0e5) &&
         finite_in_range(p.wheel_radius_m, 1.0e-5, 1.0) &&
         finite_in_range(p.wheel_offset_m, 1.0e-5, p.body_radius_m) &&
         finite_in_range(p.motor_torque_constant_nm_per_a, 1.0e-8, 100.0) &&
         finite_in_range(p.motor_back_emf_v_per_rad_s, 1.0e-8, 100.0) &&
         finite_in_range(p.motor_resistance_ohm, 1.0e-6, 1.0e4) &&
         finite_in_range(p.gear_ratio, 1.0e-3, 1.0e4) &&
         finite_in_range(p.drivetrain_efficiency, 1.0e-6, 1.0) &&
         finite_in_range(p.motor_inertia_kg_m2, 1.0e-12, 10.0) &&
         finite_in_range(p.motor_current_limit_a, 1.0e-6, 1.0e6) &&
         finite_in_range(p.motor_time_constant_s, 1.0e-7, 100.0) &&
         finite_in_range(p.tire_friction_coefficient, 0.0, 10.0) &&
         finite_in_range(p.tire_longitudinal_stiffness_n_per_mps, 1.0e-3,
                         1.0e9) &&
         finite_in_range(p.linear_drag_n_per_mps, 0.0, 1.0e6) &&
         finite_in_range(p.angular_drag_nm_per_rad_s, 0.0, 1.0e6) &&
         finite_in_range(p.battery_voltage, 1.0e-6, 1.0e6);
}

double wrap_angle(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

struct ContactResult {
  double force_x_body{0.0};
  double force_y_body{0.0};
  double longitudinal_force{0.0};
  double requested_longitudinal_force{0.0};
  double slip_mps{0.0};
  bool limited{false};
};

}  // namespace

class Plant::Impl {
 public:
  explicit Impl(const PhysicalConfig& physical)
      : config(physical),
        motor_a(motor_parameters(physical)),
        motor_b(motor_parameters(physical)) {}

  static MotorParameters motor_parameters(const PhysicalConfig& p) {
    return {p.motor_torque_constant_nm_per_a,
            p.motor_back_emf_v_per_rad_s,
            p.motor_resistance_ohm,
            p.gear_ratio,
            p.drivetrain_efficiency,
            p.motor_inertia_kg_m2,
            p.motor_current_limit_a,
            p.motor_time_constant_s,
            p.battery_voltage};
  }

  ContactResult contact(double position_x, double drive_y,
                        double wheel_speed) const {
    // Wheel-local chassis velocity. The wheel's longitudinal direction is
    // drive_y * body Y; its lateral direction is body X.
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    const double velocity_x_body = c * vx + s * vy;
    const double velocity_y_body = -s * vx + c * vy + spin * position_x;
    const double longitudinal_velocity = drive_y * velocity_y_body;
    const double slip = wheel_speed * config.wheel_radius_m -
                        longitudinal_velocity;
    const double requested_longitudinal =
        config.tire_longitudinal_stiffness_n_per_mps * slip;
    const double requested_lateral =
        -config.tire_longitudinal_stiffness_n_per_mps * velocity_x_body;
    const double normal = 0.5 * config.mass_kg * kGravityMps2;
    const double limit = config.tire_friction_coefficient * normal;
    const double requested_magnitude =
        std::hypot(requested_longitudinal, requested_lateral);
    const double scale = requested_magnitude > limit
                             ? limit / requested_magnitude
                             : 1.0;

    ContactResult result;
    result.force_x_body = requested_lateral * scale;
    result.longitudinal_force = requested_longitudinal * scale;
    result.force_y_body = drive_y * result.longitudinal_force;
    result.requested_longitudinal_force = requested_longitudinal;
    result.slip_mps = slip;
    result.limited = requested_magnitude > limit;
    return result;
  }

  void substep(double command_a, double command_b, double dt) {
    const ContactResult a =
        contact(config.wheel_offset_m, 1.0, motor_a.wheel_speed_rad_s());
    const ContactResult b =
        contact(-config.wheel_offset_m, -1.0, motor_b.wheel_speed_rad_s());

    const MotorTelemetry motor_a_result = motor_a.step(
        command_a, a.longitudinal_force * config.wheel_radius_m, dt);
    const MotorTelemetry motor_b_result = motor_b.step(
        command_b, b.longitudinal_force * config.wheel_radius_m, dt);

    const double c = std::cos(heading);
    const double s = std::sin(heading);
    const double force_x_body = a.force_x_body + b.force_x_body;
    const double force_y_body = a.force_y_body + b.force_y_body;
    const double force_x_world = c * force_x_body - s * force_y_body -
                                 config.linear_drag_n_per_mps * vx;
    const double force_y_world = s * force_x_body + c * force_y_body -
                                 config.linear_drag_n_per_mps * vy;
    const double torque =
        config.wheel_offset_m * (a.force_y_body - b.force_y_body) -
        config.angular_drag_nm_per_rad_s * spin;

    const double ax_world = force_x_world / config.mass_kg;
    const double ay_world = force_y_world / config.mass_kg;
    const double angular_acceleration =
        torque / config.moment_of_inertia_kg_m2;
    acceleration_body.body_x_mps2 = c * ax_world + s * ay_world;
    acceleration_body.body_y_mps2 = -s * ax_world + c * ay_world;
    acceleration_body.angular_rad_s2 = angular_acceleration;

    vx += ax_world * dt;
    vy += ay_world * dt;
    spin += angular_acceleration * dt;
    x += vx * dt;
    y += vy * dt;
    heading = wrap_angle(heading + spin * dt);

    telemetry_a.command = command_a;
    telemetry_a.motor_current_a = motor_a_result.current_a;
    telemetry_a.wheel_speed_rad_s = motor_a_result.wheel_speed_rad_s;
    telemetry_a.slip_mps = a.slip_mps;
    telemetry_a.requested_force_n = a.requested_longitudinal_force;
    telemetry_a.applied_force_n = a.longitudinal_force;
    telemetry_a.normal_force_n = 0.5 * config.mass_kg * kGravityMps2;
    telemetry_a.traction_limited = a.limited;
    telemetry_b.command = command_b;
    telemetry_b.motor_current_a = motor_b_result.current_a;
    telemetry_b.wheel_speed_rad_s = motor_b_result.wheel_speed_rad_s;
    telemetry_b.slip_mps = b.slip_mps;
    telemetry_b.requested_force_n = b.requested_longitudinal_force;
    telemetry_b.applied_force_n = b.longitudinal_force;
    telemetry_b.normal_force_n = telemetry_a.normal_force_n;
    telemetry_b.traction_limited = b.limited;
  }

  PhysicalConfig config;
  Motor motor_a;
  Motor motor_b;
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double vx{0.0};
  double vy{0.0};
  double spin{0.0};
  PlantAcceleration acceleration_body{};
  WheelTelemetry telemetry_a{};
  WheelTelemetry telemetry_b{};
};

Plant::Plant(const PhysicalConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
  std::size_t count = 0;
  if (!supported_physical_config(config) ||
      !substep_count(config, 0.00025, count)) {
    throw std::invalid_argument("unsupported physical configuration");
  }
}
Plant::~Plant() = default;
Plant::Plant(Plant&&) noexcept = default;
Plant& Plant::operator=(Plant&&) noexcept = default;

void Plant::reset(const ResetState& state) {
  if (!supported_reset_state(state)) {
    throw std::invalid_argument("reset state exceeds supported numerical bounds");
  }
  impl_->x = state.x_m;
  impl_->y = state.y_m;
  impl_->heading = wrap_angle(state.heading_rad);
  impl_->vx = state.vx_mps;
  impl_->vy = state.vy_mps;
  impl_->spin = state.spin_rad_s;
  impl_->acceleration_body = {};
  const double wheel_speed =
      state.spin_rad_s * impl_->config.wheel_offset_m /
      impl_->config.wheel_radius_m;
  impl_->motor_a.reset(wheel_speed);
  impl_->motor_b.reset(wheel_speed);
  impl_->telemetry_a = {};
  impl_->telemetry_b = {};
  impl_->telemetry_a.wheel_speed_rad_s = wheel_speed;
  impl_->telemetry_b.wheel_speed_rad_s = wheel_speed;
  impl_->telemetry_a.normal_force_n =
      0.5 * impl_->config.mass_kg * kGravityMps2;
  impl_->telemetry_b.normal_force_n = impl_->telemetry_a.normal_force_n;
}

void Plant::step(double wheel_a_command, double wheel_b_command, double dt_s) {
  // Keep the stiff tire/rotor coupling well inside its explicit stability
  // region even when a caller selects a relatively coarse control tick.
  if (!std::isfinite(wheel_a_command) || !std::isfinite(wheel_b_command)) {
    throw std::invalid_argument("motor commands must be finite");
  }
  std::size_t substeps = 0;
  if (!substep_count(impl_->config, dt_s, substeps)) {
    throw std::invalid_argument("physical timestep exceeds bounded substep budget");
  }
  const double substep_s = dt_s / static_cast<double>(substeps);
  for (std::size_t i = 0; i < substeps; ++i) {
    impl_->substep(wheel_a_command, wheel_b_command, substep_s);
    const Snapshot state = snapshot(0);
    const PlantAcceleration accel = acceleration();
    const bool finite_state =
        std::isfinite(state.x_m) && std::isfinite(state.y_m) &&
        std::isfinite(state.heading_rad) && std::isfinite(state.vx_mps) &&
        std::isfinite(state.vy_mps) && std::isfinite(state.spin_rad_s) &&
        std::isfinite(state.wheel_a.motor_current_a) &&
        std::isfinite(state.wheel_b.motor_current_a) &&
        std::isfinite(state.wheel_a.wheel_speed_rad_s) &&
        std::isfinite(state.wheel_b.wheel_speed_rad_s) &&
        std::isfinite(state.wheel_a.slip_mps) &&
        std::isfinite(state.wheel_b.slip_mps) &&
        std::isfinite(state.wheel_a.requested_force_n) &&
        std::isfinite(state.wheel_b.requested_force_n) &&
        std::isfinite(state.wheel_a.applied_force_n) &&
        std::isfinite(state.wheel_b.applied_force_n) &&
        std::isfinite(accel.body_x_mps2) &&
        std::isfinite(accel.body_y_mps2) &&
        std::isfinite(accel.angular_rad_s2);
    if (!finite_state) {
      throw std::overflow_error("physical state became non-finite");
    }
  }
}

bool Plant::substep_count(const PhysicalConfig& config, double dt_s,
                          std::size_t& count) noexcept {
  count = 0;
  if (!supported_physical_config(config) || !std::isfinite(dt_s) ||
      dt_s <= 0.0) {
    return false;
  }
  const double reflected_inertia =
      config.motor_inertia_kg_m2 * config.gear_ratio * config.gear_ratio;
  const double denominator =
      config.tire_longitudinal_stiffness_n_per_mps *
      config.wheel_radius_m * config.wheel_radius_m;
  const double wheel_coupling_time = reflected_inertia / denominator;
  const double chassis_coupling_time =
      config.mass_kg /
      (2.0 * config.tire_longitudinal_stiffness_n_per_mps);
  const double angular_denominator =
      2.0 * config.tire_longitudinal_stiffness_n_per_mps *
      config.wheel_offset_m * config.wheel_offset_m;
  const double angular_coupling_time =
      config.moment_of_inertia_kg_m2 / angular_denominator;
  const double electrical_mechanical_time =
      config.motor_inertia_kg_m2 * config.motor_resistance_ohm /
      (config.motor_torque_constant_nm_per_a *
       config.motor_back_emf_v_per_rad_s * config.drivetrain_efficiency);
  const double linear_drag_time =
      config.linear_drag_n_per_mps > 0.0
          ? config.mass_kg / config.linear_drag_n_per_mps
          : std::numeric_limits<double>::infinity();
  const double angular_drag_time =
      config.angular_drag_nm_per_rad_s > 0.0
          ? config.moment_of_inertia_kg_m2 /
                config.angular_drag_nm_per_rad_s
          : std::numeric_limits<double>::infinity();
  const double maximum_substep_s =
      std::min({0.00025, 0.1 * wheel_coupling_time,
                0.1 * chassis_coupling_time,
                0.1 * angular_coupling_time,
                0.1 * config.motor_time_constant_s,
                0.1 * electrical_mechanical_time,
                0.1 * linear_drag_time, 0.1 * angular_drag_time});
  if (!std::isfinite(maximum_substep_s) || maximum_substep_s <= 0.0) {
    return false;
  }
  const double required = std::ceil(dt_s / maximum_substep_s);
  if (!std::isfinite(required) || required < 1.0 ||
      required > static_cast<double>(kMaximumSubsteps)) {
    return false;
  }
  count = static_cast<std::size_t>(required);
  return true;
}

Snapshot Plant::snapshot(Micros now_us) const {
  Snapshot result;
  result.time_us = now_us;
  result.x_m = impl_->x;
  result.y_m = impl_->y;
  result.heading_rad = impl_->heading;
  result.vx_mps = impl_->vx;
  result.vy_mps = impl_->vy;
  result.spin_rad_s = impl_->spin;
  result.wheel_a = impl_->telemetry_a;
  result.wheel_b = impl_->telemetry_b;
  return result;
}

PlantAcceleration Plant::acceleration() const {
  return impl_->acceleration_body;
}

}  // namespace melty::sim
