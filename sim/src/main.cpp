#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "sim/simulator.hpp"

namespace {

struct Options {
  double duration_s{5.0};
  std::string output{"simulation.csv"};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--duration" && i + 1 < argc) {
      options.duration_s = std::stod(argv[++i]);
    } else if (argument == "--output" && i + 1 < argc) {
      options.output = argv[++i];
    } else if (argument == "--help") {
      std::cout << "usage: melty_sim [--duration SECONDS] [--output CSV|-]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown or incomplete argument: " + argument);
    }
  }
  if (!(options.duration_s > 0.0) || options.duration_s > 3600.0) {
    throw std::invalid_argument("duration must be in (0, 3600] seconds");
  }
  return options;
}

void write_row(std::ostream& output, const melty::sim::Snapshot& s) {
  output << s.time_us << ',' << s.x_m << ',' << s.y_m << ','
         << s.heading_rad << ',' << s.vx_mps << ',' << s.vy_mps << ','
         << s.spin_rad_s << ',' << s.firmware.controller.spin_rad_s << ','
         << s.firmware.controller.phase_rad << ','
         << s.firmware.output.wheel_a << ',' << s.firmware.output.wheel_b
         << ',' << s.wheel_a.motor_current_a << ','
         << s.wheel_b.motor_current_a << ',' << s.wheel_a.slip_mps << ','
         << s.wheel_b.slip_mps << ','
         << static_cast<unsigned>(s.firmware.faults) << '\n';
}

void write_recording(std::ostream& out,
                     const melty::sim::SimulationConfig& c,
                     const melty::sim::ResetState& state,
                     melty::Micros requested_duration_us,
                     melty::Micros simulated_duration_us) {
  const auto& p = c.physical;
  const auto& s = c.sensor;
  const auto& r = c.firmware;
  const auto& k = r.controller;
  out << std::setprecision(17) << std::boolalpha
      << "{\n  \"schema\": \"meltybrain-simulator-recording\",\n"
      << "  \"version\": 1,\n  \"config\": {\n"
      << "    \"physical\": {\n"
      << "      \"mass_kg\": " << p.mass_kg << ",\n"
      << "      \"body_radius_m\": " << p.body_radius_m << ",\n"
      << "      \"moment_of_inertia_kg_m2\": " << p.moment_of_inertia_kg_m2 << ",\n"
      << "      \"wheel_radius_m\": " << p.wheel_radius_m << ",\n"
      << "      \"wheel_offset_m\": " << p.wheel_offset_m << ",\n"
      << "      \"motor_torque_constant_nm_per_a\": " << p.motor_torque_constant_nm_per_a << ",\n"
      << "      \"motor_back_emf_v_per_rad_s\": " << p.motor_back_emf_v_per_rad_s << ",\n"
      << "      \"motor_resistance_ohm\": " << p.motor_resistance_ohm << ",\n"
      << "      \"gear_ratio\": " << p.gear_ratio << ",\n"
      << "      \"drivetrain_efficiency\": " << p.drivetrain_efficiency << ",\n"
      << "      \"motor_inertia_kg_m2\": " << p.motor_inertia_kg_m2 << ",\n"
      << "      \"motor_current_limit_a\": " << p.motor_current_limit_a << ",\n"
      << "      \"motor_time_constant_s\": " << p.motor_time_constant_s << ",\n"
      << "      \"tire_friction_coefficient\": " << p.tire_friction_coefficient << ",\n"
      << "      \"tire_longitudinal_stiffness_n_per_mps\": " << p.tire_longitudinal_stiffness_n_per_mps << ",\n"
      << "      \"linear_drag_n_per_mps\": " << p.linear_drag_n_per_mps << ",\n"
      << "      \"angular_drag_nm_per_rad_s\": " << p.angular_drag_nm_per_rad_s << ",\n"
      << "      \"battery_voltage\": " << p.battery_voltage << "\n    },\n"
      << "    \"sensor\": {\n"
      << "      \"radius_m\": " << s.radius_m << ",\n"
      << "      \"angle_rad\": " << s.angle_rad << ",\n"
      << "      \"max_acceleration_mps2\": " << s.max_acceleration_mps2 << ",\n"
      << "      \"noise_stddev_mps2\": " << s.noise_stddev_mps2 << ",\n"
      << "      \"bias_x_mps2\": " << s.bias_x_mps2 << ",\n"
      << "      \"bias_y_mps2\": " << s.bias_y_mps2 << ",\n"
      << "      \"sample_period_us\": " << s.sample_period_us << ",\n"
      << "      \"latency_us\": " << s.latency_us << "\n    },\n"
      << "    \"firmware\": {\n"
      << "      \"control_period_us\": " << r.control_period_us << ",\n"
      << "      \"acceleration_timeout_us\": " << r.acceleration_timeout_us << ",\n"
      << "      \"rc_timeout_us\": " << r.rc_timeout_us << ",\n"
      << "      \"maximum_tick_interval_us\": " << r.maximum_tick_interval_us << ",\n"
      << "      \"arm_spin_max\": " << r.arm_spin_max << ",\n"
      << "      \"arm_confirm_ticks\": " << r.arm_confirm_ticks << ",\n"
      << "      \"controller\": {\n"
      << "        \"sensor_radius_m\": " << k.sensor_radius_m << ",\n"
      << "        \"sensor_angle_rad\": " << k.sensor_angle_rad << ",\n"
      << "        \"spin_direction\": " << k.spin_direction << ",\n"
      << "        \"minimum_phase_spin_rad_s\": " << k.minimum_phase_spin_rad_s << ",\n"
      << "        \"radial_accel_floor_mps2\": " << k.radial_accel_floor_mps2 << ",\n"
      << "        \"radial_accel_filter_hz\": " << k.radial_accel_filter_hz << ",\n"
      << "        \"maximum_spin_rad_s\": " << k.maximum_spin_rad_s << ",\n"
      << "        \"spin_kp\": " << k.spin_kp << ",\n"
      << "        \"spin_ki\": " << k.spin_ki << ",\n"
      << "        \"spin_integrator_limit\": " << k.spin_integrator_limit << ",\n"
      << "        \"command_headroom\": " << k.command_headroom << ",\n"
      << "        \"translation_gain\": " << k.translation_gain << ",\n"
      << "        \"translation_phase_offset_rad\": " << k.translation_phase_offset_rad
      << "\n      }\n    },\n"
      << "    \"physics_tick_us\": " << c.physics_tick_us << ",\n"
      << "    \"command_latency_us\": " << c.command_latency_us << ",\n"
      << "    \"actuator_latency_us\": " << c.actuator_latency_us << ",\n"
      << "    \"scenario_seed\": " << c.scenario_seed << "\n  },\n"
      << "  \"initial_state\": {\n"
      << "    \"x_m\": " << state.x_m << ", \"y_m\": " << state.y_m << ",\n"
      << "    \"heading_rad\": " << state.heading_rad << ",\n"
      << "    \"vx_mps\": " << state.vx_mps << ", \"vy_mps\": " << state.vy_mps << ",\n"
      << "    \"spin_rad_s\": " << state.spin_rad_s << ",\n"
      << "    \"estimated_phase_rad\": " << state.estimated_phase_rad << "\n  },\n"
      << "  \"requested_duration_us\": " << requested_duration_us << ",\n"
      << "  \"simulated_duration_us\": " << simulated_duration_us << ",\n"
      << "  \"command_events\": [\n"
      << "    {\"time_us\": 0, \"spin\": 0, \"translate_x\": 0, \"translate_y\": 0, \"arm\": false, \"reset_phase\": false},\n"
      << "    {\"time_us\": 10000, \"spin\": 0.25, \"translate_x\": 0, \"translate_y\": 0, \"arm\": true, \"reset_phase\": false}";
  if (requested_duration_us > 1'000'000) {
    out << ",\n    {\"time_us\": 1010000, \"spin\": 0.25, \"translate_x\": 0.65, \"translate_y\": 0, \"arm\": true, \"reset_phase\": false}";
  }
  out << "\n  ],\n"
      << "  \"notes\": \"Illustrative unmeasured parameters; phase offset is a single-speed calibration, not truth feedback.\"\n"
      << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    melty::sim::SimulationConfig config{};
    // The illustrative drivetrain has appreciable motor/tire phase lag at the
    // reference spin. This measured reference compensation is intentionally a
    // firmware calibration parameter, not simulator truth feedback.
    config.firmware.controller.translation_phase_offset_rad = 2.15;
    const melty::sim::ResetState initial_state{};
    melty::sim::Simulator simulator(config);
    simulator.reset(initial_state);

    std::ofstream file;
    std::ostream* output = &std::cout;
    if (options.output != "-") {
      file.open(options.output);
      if (!file) {
        throw std::runtime_error("could not open output: " + options.output);
      }
      output = &file;
    }
    *output << std::setprecision(12);
    *output << "time_us,x_m,y_m,heading_rad,vx_mps,vy_mps,spin_rad_s,"
               "estimated_spin_rad_s,estimated_phase_rad,command_a,command_b,current_a,current_b,"
               "slip_a_mps,slip_b_mps,fault_bits\n";

    // First present the required low-spin disarmed frames, then request a
    // reachable reference spin. Translation starts after the estimator has had
    // time to establish phase from the physical radial acceleration.
    simulator.set_command({0.0, 0.0, 0.0, false, false});
    simulator.advance_for(10'000);
    simulator.set_command({0.25, 0.0, 0.0, true, false});

    const melty::Micros duration_us =
        static_cast<melty::Micros>(options.duration_s * 1.0e6);
    const melty::Micros row_period_us = 10'000;
    melty::Micros elapsed = 0;
    for (; elapsed < duration_us;) {
      if (elapsed >= 1'000'000) {
        simulator.set_command({0.25, 0.65, 0.0, true, false});
      }
      const melty::Micros step =
          std::min(row_period_us, duration_us - elapsed);
      // CLI durations are rounded down to the configured physical tick.
      const melty::Micros aligned =
          step - step % config.physics_tick_us;
      if (aligned == 0) break;
      simulator.advance_for(aligned);
      elapsed += aligned;
      write_row(*output, simulator.snapshot());
    }

    if (options.output != "-") {
      std::ofstream metadata(options.output + ".json");
      if (!metadata) {
        throw std::runtime_error("could not open recording sidecar: " +
                                 options.output + ".json");
      }
      write_recording(metadata, config, initial_state, duration_us,
                      10'000 + elapsed);
      metadata.close();
      if (!metadata) {
        throw std::runtime_error("could not write recording sidecar: " +
                                 options.output + ".json");
      }
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "melty_sim: " << error.what() << '\n';
    return 2;
  }
}
