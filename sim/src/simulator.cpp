#include "sim/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "sim/plant.hpp"
#include "sim/sim_hal.hpp"

namespace melty::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaximumAdvanceTicks = 10'000'000;

bool finite(double value) { return std::isfinite(value); }
bool positive(double value) { return finite(value) && value > 0.0; }
bool nonnegative(double value) { return finite(value) && value >= 0.0; }

template <typename T>
struct Delayed {
  Micros delivery_us;
  T value;
};

class DeterministicNoise {
 public:
  void seed(std::uint64_t seed) {
    state_ = seed == 0 ? 0x9e3779b97f4a7c15ULL : seed;
    spare_available_ = false;
    spare_ = 0.0;
  }

  double normal() {
    if (spare_available_) {
      spare_available_ = false;
      return spare_;
    }
    const double u1 = std::max(uniform(), std::numeric_limits<double>::min());
    const double u2 = uniform();
    const double radius = std::sqrt(-2.0 * std::log(u1));
    const double angle = 2.0 * kPi * u2;
    spare_ = radius * std::sin(angle);
    spare_available_ = true;
    return radius * std::cos(angle);
  }

 private:
  std::uint64_t next() {
    std::uint64_t x = state_;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    state_ = x;
    return x * 2685821657736338717ULL;
  }

  double uniform() {
    return static_cast<double>(next() >> 11) * 0x1.0p-53;
  }

  std::uint64_t state_{1};
  bool spare_available_{false};
  double spare_{0.0};
};

std::string validation_message(const std::vector<std::string>& errors) {
  std::ostringstream stream;
  stream << "invalid simulation configuration";
  for (const auto& error : errors) {
    stream << "; " << error;
  }
  return stream.str();
}

}  // namespace

class Simulator::Impl {
 public:
  explicit Impl(const SimulationConfig& configuration)
      : config(configuration), plant(config.physical), runtime(hal, config.firmware) {
    reset({});
  }

  static Micros checked_add(Micros lhs, Micros rhs) {
    if (rhs > std::numeric_limits<Micros>::max() - lhs) {
      throw std::overflow_error("simulated timestamp overflow");
    }
    return lhs + rhs;
  }

  AccelerationSample acquire_acceleration(Micros acquisition_us) {
    const PlantAcceleration center = plant.acceleration();
    const Snapshot truth = plant.snapshot(now_us);
    const double angle = config.sensor.angle_rad;
    const double c = std::cos(angle);
    const double s = std::sin(angle);

    // The sensor sits on body +X at radius_m. angle_rad rotates its axes from
    // the body axes and is independent of that physical position.
    const double point_x_body =
        center.body_x_mps2 - truth.spin_rad_s * truth.spin_rad_s *
                                     config.sensor.radius_m;
    const double point_y_body =
        center.body_y_mps2 + center.angular_rad_s2 * config.sensor.radius_m;
    double sensor_x = c * point_x_body + s * point_y_body;
    double sensor_y = -s * point_x_body + c * point_y_body;
    if (!finite(sensor_x) || !finite(sensor_y)) {
      throw std::overflow_error("sensor kinematics became non-finite");
    }
    sensor_x += config.sensor.bias_x_mps2 +
                config.sensor.noise_stddev_mps2 * noise.normal();
    sensor_y += config.sensor.bias_y_mps2 +
                config.sensor.noise_stddev_mps2 * noise.normal();
    if (!finite(sensor_x) || !finite(sensor_y)) {
      throw std::overflow_error("sensor model became non-finite");
    }
    const bool saturated =
        std::abs(sensor_x) > config.sensor.max_acceleration_mps2 ||
        std::abs(sensor_y) > config.sensor.max_acceleration_mps2;
    sensor_x = std::clamp(sensor_x, -config.sensor.max_acceleration_mps2,
                          config.sensor.max_acceleration_mps2);
    sensor_y = std::clamp(sensor_y, -config.sensor.max_acceleration_mps2,
                          config.sensor.max_acceleration_mps2);
    return {sensor_x, sensor_y, saturated, acquisition_us};
  }

  RcCommand acquire_command(Micros acquisition_us) const {
    return {desired_command.spin,
            desired_command.translate_x,
            desired_command.translate_y,
            desired_command.arm,
            desired_command.reset_phase,
            acquisition_us};
  }

  void enqueue_sensor(Micros acquisition_us) {
    AccelerationSample sample = acquire_acceleration(acquisition_us);
    sensor_queue.push_back(
        {checked_add(acquisition_us, config.sensor.latency_us), sample});
  }

  void enqueue_command(Micros acquisition_us) {
    command_queue.push_back(
        {checked_add(acquisition_us, config.command_latency_us),
         acquire_command(acquisition_us)});
  }

  void deliver_due() {
    while (!sensor_queue.empty() && sensor_queue.front().delivery_us <= now_us) {
      hal.publish_acceleration(sensor_queue.front().value);
      sensor_queue.pop_front();
    }
    while (!command_queue.empty() &&
           command_queue.front().delivery_us <= now_us) {
      hal.publish_rc(command_queue.front().value);
      command_queue.pop_front();
    }
    while (!actuator_queue.empty() &&
           actuator_queue.front().delivery_us <= now_us) {
      applied_motor_output = actuator_queue.front().value;
      actuator_queue.pop_front();
    }
  }

  void advance_one(Micros dt_us) {
    const MotorOutput prior_output = applied_motor_output;
    plant.step(prior_output.wheel_a, prior_output.wheel_b,
               static_cast<double>(dt_us) * 1.0e-6);
    now_us = checked_add(now_us, dt_us);
    hal.set_now(now_us);

    if (now_us == next_sensor_us) {
      enqueue_sensor(now_us);
      next_sensor_us = checked_add(next_sensor_us,
                                   config.sensor.sample_period_us);
    }
    if (now_us == next_control_us) {
      enqueue_command(now_us);
      deliver_due();
      const bool was_armed = runtime.status().armed;
      runtime.tick();
      if (was_armed && !runtime.status().armed) {
        last_disarm_us = now_us;
        last_disarm_faults = runtime.status().faults;
      }
      actuator_queue.push_back(
          {checked_add(now_us, config.actuator_latency_us),
           hal.motor_output()});
      deliver_due();
      next_control_us = checked_add(next_control_us,
                                    config.firmware.control_period_us);
    } else {
      deliver_due();
    }
  }

  void reset(const ResetState& state) {
    plant.reset(state);
    hal.reset();
    now_us = 0;
    hal.set_now(0);
    desired_command = {};
    last_disarm_us = 0;
    last_disarm_faults = Fault::none;
    sensor_queue.clear();
    command_queue.clear();
    actuator_queue.clear();
    applied_motor_output = {};
    noise.seed(config.scenario_seed);
    enqueue_sensor(0);
    enqueue_command(0);
    next_sensor_us = config.sensor.sample_period_us;
    next_control_us = config.firmware.control_period_us;
    deliver_due();
    runtime.reset(state.estimated_phase_rad);
  }

  SimulationConfig config;
  Plant plant;
  SimHal hal;
  Runtime runtime;
  Micros now_us{0};
  Micros next_sensor_us{0};
  Micros next_control_us{0};
  UserCommand desired_command{};
  Micros last_disarm_us{0};
  Fault last_disarm_faults{Fault::none};
  std::deque<Delayed<AccelerationSample>> sensor_queue;
  std::deque<Delayed<RcCommand>> command_queue;
  std::deque<Delayed<MotorOutput>> actuator_queue;
  MotorOutput applied_motor_output{};
  DeterministicNoise noise;
};

Simulator::Simulator(const SimulationConfig& config) {
  const auto errors = validate(config);
  if (!errors.empty()) {
    throw std::invalid_argument(validation_message(errors));
  }
  impl_ = std::make_unique<Impl>(config);
}

Simulator::~Simulator() = default;
Simulator::Simulator(Simulator&&) noexcept = default;
Simulator& Simulator::operator=(Simulator&&) noexcept = default;

std::vector<std::string> Simulator::validate(const SimulationConfig& config) {
  std::vector<std::string> errors;
  const PhysicalConfig& p = config.physical;
  if (!positive(p.mass_kg)) errors.emplace_back("physical.mass_kg must be positive and finite");
  if (!positive(p.body_radius_m)) errors.emplace_back("physical.body_radius_m must be positive and finite");
  if (!positive(p.moment_of_inertia_kg_m2)) errors.emplace_back("physical.moment_of_inertia_kg_m2 must be positive and finite");
  if (!positive(p.wheel_radius_m)) errors.emplace_back("physical.wheel_radius_m must be positive and finite");
  if (!positive(p.wheel_offset_m) ||
      (finite(p.body_radius_m) && p.wheel_offset_m > p.body_radius_m)) {
    errors.emplace_back("physical.wheel_offset_m must be positive, finite, and no larger than body radius");
  }
  if (!positive(p.motor_torque_constant_nm_per_a)) errors.emplace_back("physical.motor_torque_constant_nm_per_a must be positive and finite");
  if (!positive(p.motor_back_emf_v_per_rad_s)) errors.emplace_back("physical.motor_back_emf_v_per_rad_s must be positive and finite");
  if (!positive(p.motor_resistance_ohm)) errors.emplace_back("physical.motor_resistance_ohm must be positive and finite");
  if (!positive(p.gear_ratio)) errors.emplace_back("physical.gear_ratio must be positive and finite");
  if (!positive(p.drivetrain_efficiency) || p.drivetrain_efficiency > 1.0) errors.emplace_back("physical.drivetrain_efficiency must be in (0, 1]");
  if (!positive(p.motor_inertia_kg_m2)) errors.emplace_back("physical.motor_inertia_kg_m2 must be positive and finite");
  if (!positive(p.motor_current_limit_a)) errors.emplace_back("physical.motor_current_limit_a must be positive and finite");
  if (!positive(p.motor_time_constant_s)) errors.emplace_back("physical.motor_time_constant_s must be positive and finite");
  if (!nonnegative(p.tire_friction_coefficient)) errors.emplace_back("physical.tire_friction_coefficient must be nonnegative and finite");
  if (!positive(p.tire_longitudinal_stiffness_n_per_mps)) errors.emplace_back("physical.tire_longitudinal_stiffness_n_per_mps must be positive and finite");
  if (!nonnegative(p.linear_drag_n_per_mps)) errors.emplace_back("physical.linear_drag_n_per_mps must be nonnegative and finite");
  if (!nonnegative(p.angular_drag_nm_per_rad_s)) errors.emplace_back("physical.angular_drag_nm_per_rad_s must be nonnegative and finite");
  if (!positive(p.battery_voltage)) errors.emplace_back("physical.battery_voltage must be positive and finite");

  const SensorModelConfig& sensor = config.sensor;
  if (!nonnegative(sensor.radius_m) || sensor.radius_m > 10.0) errors.emplace_back("sensor.radius_m must be in [0, 10]");
  if (!finite(sensor.angle_rad) || std::abs(sensor.angle_rad) > 1.0e6) errors.emplace_back("sensor.angle_rad exceeds supported numerical bounds");
  if (!positive(sensor.max_acceleration_mps2) || sensor.max_acceleration_mps2 > 1.0e9) errors.emplace_back("sensor.max_acceleration_mps2 must be in (0, 1e9]");
  if (!nonnegative(sensor.noise_stddev_mps2) || sensor.noise_stddev_mps2 > 1.0e8) errors.emplace_back("sensor.noise_stddev_mps2 must be in [0, 1e8]");
  if (!finite(sensor.bias_x_mps2) || !finite(sensor.bias_y_mps2) ||
      std::abs(sensor.bias_x_mps2) > 1.0e9 ||
      std::abs(sensor.bias_y_mps2) > 1.0e9) errors.emplace_back("sensor biases exceed supported numerical bounds");
  if (sensor.sample_period_us == 0) errors.emplace_back("sensor.sample_period_us must be nonzero");
  if (config.physics_tick_us == 0 || config.physics_tick_us > 5'000) errors.emplace_back("physics_tick_us must be in [1, 5000]");
  constexpr Micros kMaximumTransportLatencyUs = 10'000'000;
  if (sensor.latency_us > kMaximumTransportLatencyUs ||
      config.command_latency_us > kMaximumTransportLatencyUs ||
      config.actuator_latency_us > kMaximumTransportLatencyUs) {
    errors.emplace_back("transport latencies must not exceed 10 seconds");
  }
  constexpr std::uint64_t kMaximumQueuedEvents = 100'000;
  const auto queued_events = [](Micros latency, Micros period) {
    if (period == 0) return std::numeric_limits<std::uint64_t>::max();
    return latency / period + (latency % period != 0 ? 1u : 0u) + 1u;
  };
  const std::uint64_t sensor_events =
      queued_events(sensor.latency_us, sensor.sample_period_us);
  const std::uint64_t command_events =
      queued_events(config.command_latency_us,
                    config.firmware.control_period_us);
  const std::uint64_t actuator_events =
      queued_events(config.actuator_latency_us,
                    config.firmware.control_period_us);
  std::uint64_t total_events = 0;
  const auto exceeds_event_budget = [&](std::uint64_t count) {
    if (count > kMaximumQueuedEvents - total_events) return true;
    total_events += count;
    return false;
  };
  if (exceeds_event_budget(sensor_events) ||
      exceeds_event_budget(command_events) ||
      exceeds_event_budget(actuator_events)) {
    errors.emplace_back("configured transport queues exceed 100000 in-flight events");
  }
  if (config.physics_tick_us != 0 &&
      sensor.sample_period_us % config.physics_tick_us != 0) {
    errors.emplace_back("sensor.sample_period_us must be a multiple of physics_tick_us");
  }
  if (config.physics_tick_us != 0 &&
      (config.firmware.control_period_us < config.physics_tick_us ||
       config.firmware.control_period_us % config.physics_tick_us != 0)) {
    errors.emplace_back("firmware.control_period_us must be at least and a multiple of physics_tick_us");
  }
  if (!Runtime::valid_config(config.firmware)) {
    errors.emplace_back("firmware runtime configuration is invalid");
  }
  const ControllerConfig& controller = config.firmware.controller;
  const bool supported_firmware =
      controller.sensor_radius_m >= 1.0e-5 &&
      controller.sensor_radius_m <= 10.0 &&
      std::abs(controller.sensor_angle_rad) <= 1.0e6 &&
      controller.minimum_phase_spin_rad_s <= 1.0e5 &&
      controller.radial_accel_floor_mps2 <= 1.0e9 &&
      controller.radial_accel_filter_hz >= 1.0e-3 &&
      controller.radial_accel_filter_hz <= 1.0e5 &&
      controller.maximum_spin_rad_s <= 1.0e5 &&
      controller.spin_kp <= 1.0e4 && controller.spin_ki <= 1.0e4 &&
      controller.spin_integrator_limit <= 1.0 &&
      controller.translation_gain <= 10.0 &&
      std::abs(controller.translation_phase_offset_rad) <= 1.0e6 &&
      config.firmware.control_period_us <= 1'000'000'000ULL &&
      config.firmware.acceleration_timeout_us <= 1'000'000'000ULL &&
      config.firmware.rc_timeout_us <= 1'000'000'000ULL &&
      config.firmware.maximum_tick_interval_us <= 1'000'000'000ULL &&
      config.firmware.arm_confirm_ticks <= 1'000'000;
  if (!supported_firmware) {
    errors.emplace_back("firmware values exceed supported simulation bounds");
  }
  std::size_t substeps = 0;
  if (config.physics_tick_us != 0 &&
      !Plant::substep_count(p,
                            static_cast<double>(config.physics_tick_us) * 1.0e-6,
                            substeps)) {
    errors.emplace_back("physical values exceed supported bounds or require more than 100 integration substeps");
  }
  return errors;
}

const SimulationConfig& Simulator::config() const { return impl_->config; }

void Simulator::reset(const ResetState& state) {
  if (!finite(state.x_m) || !finite(state.y_m) ||
      !finite(state.heading_rad) || !finite(state.vx_mps) ||
      !finite(state.vy_mps) || !finite(state.spin_rad_s) ||
      !finite(state.estimated_phase_rad) || std::abs(state.x_m) > 1.0e6 ||
      std::abs(state.y_m) > 1.0e6 || std::abs(state.heading_rad) > 1.0e6 ||
      std::abs(state.vx_mps) > 1.0e3 || std::abs(state.vy_mps) > 1.0e3 ||
      std::abs(state.spin_rad_s) > 1.0e4 ||
      std::abs(state.estimated_phase_rad) > 1.0e6) {
    throw std::invalid_argument("reset state exceeds supported numerical bounds");
  }
  impl_->reset(state);
}

void Simulator::set_command(const UserCommand& command) {
  if (!finite(command.spin) || !finite(command.translate_x) ||
      !finite(command.translate_y) || command.spin < 0.0 ||
      command.spin > 1.0 || command.translate_x < -1.0 ||
      command.translate_x > 1.0 || command.translate_y < -1.0 ||
      command.translate_y > 1.0) {
    throw std::invalid_argument(
        "command spin must be in [0,1] and translation axes in [-1,1]");
  }
  impl_->desired_command = command;
}

void Simulator::advance_ticks(std::size_t ticks) {
  if (ticks > kMaximumAdvanceTicks) {
    throw std::length_error("advance request exceeds simulator safety limit");
  }
  if (ticks != 0 &&
      ticks > (std::numeric_limits<Micros>::max() - impl_->now_us) /
                  impl_->config.physics_tick_us) {
    throw std::overflow_error("simulated timestamp overflow");
  }
  for (std::size_t i = 0; i < ticks; ++i) {
    impl_->advance_one(impl_->config.physics_tick_us);
  }
}

void Simulator::advance_for(Micros duration_us) {
  if (duration_us % impl_->config.physics_tick_us != 0) {
    throw std::invalid_argument(
        "advance_for duration must be a multiple of physics_tick_us");
  }
  const Micros ticks = duration_us / impl_->config.physics_tick_us;
  if (ticks > std::numeric_limits<std::size_t>::max()) {
    throw std::length_error("advance request exceeds platform size limit");
  }
  advance_ticks(static_cast<std::size_t>(ticks));
}

Snapshot Simulator::snapshot() const {
  Snapshot result = impl_->plant.snapshot(impl_->now_us);
  if (const AccelerationSample* delivered =
          impl_->hal.delivered_acceleration()) {
    result.sensed_acceleration = *delivered;
  }
  result.firmware = impl_->runtime.status();
  result.consumed_acceleration = impl_->hal.last_read_acceleration();
  result.consumed_command = impl_->hal.last_read_rc();
  result.last_disarm_us = impl_->last_disarm_us;
  result.last_disarm_faults = impl_->last_disarm_faults;
  return result;
}

}  // namespace melty::sim
