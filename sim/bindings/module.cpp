#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "melty/runtime.hpp"
#include "sim/simulator.hpp"

namespace py = pybind11;
using namespace melty;
using namespace melty::sim;

#define FIELD(type, name) .def_readwrite(#name, &type::name)

PYBIND11_MODULE(melty_sim, m) {
  m.doc() = "Deterministic meltybrain firmware and planar simulator";

  py::enum_<Fault>(m, "Fault", py::arithmetic())
      .value("NONE", Fault::none)
      .value("INVALID_CONFIGURATION", Fault::invalid_configuration)
      .value("INVALID_ACCELERATION", Fault::invalid_acceleration)
      .value("STALE_ACCELERATION", Fault::stale_acceleration)
      .value("INVALID_RC", Fault::invalid_rc)
      .value("STALE_RC", Fault::stale_rc)
      .value("CONTROL_DEADLINE", Fault::control_deadline)
      .value("PHASE_INVALID", Fault::phase_invalid)
      .value("ACCELERATION_SATURATED", Fault::acceleration_saturated)
      .value("CONTROLLER_NUMERIC", Fault::controller_numeric)
      .value("HAL_ERROR", Fault::hal_error);

  py::class_<AccelerationSample>(m, "AccelerationSample")
      .def(py::init<>())
      FIELD(AccelerationSample, x_mps2)
      FIELD(AccelerationSample, y_mps2)
      FIELD(AccelerationSample, saturated)
      FIELD(AccelerationSample, timestamp_us);
  py::class_<RcCommand>(m, "RcCommand")
      .def(py::init<>())
      FIELD(RcCommand, spin)
      FIELD(RcCommand, translate_x)
      FIELD(RcCommand, translate_y)
      FIELD(RcCommand, arm)
      FIELD(RcCommand, reset_phase)
      FIELD(RcCommand, hardware_enabled)
      FIELD(RcCommand, timestamp_us);
  py::class_<MotorOutput>(m, "MotorOutput")
      .def(py::init<>()) FIELD(MotorOutput, wheel_a) FIELD(MotorOutput, wheel_b);
  py::class_<ControllerTelemetry>(m, "ControllerTelemetry")
      .def(py::init<>())
      FIELD(ControllerTelemetry, phase_rad)
      FIELD(ControllerTelemetry, spin_rad_s)
      FIELD(ControllerTelemetry, radial_accel_mps2)
      FIELD(ControllerTelemetry, spin_error_rad_s)
      FIELD(ControllerTelemetry, spin_integrator)
      FIELD(ControllerTelemetry, common_command)
      FIELD(ControllerTelemetry, modulation_command)
      FIELD(ControllerTelemetry, phase_valid);
  py::class_<RuntimeStatus>(m, "RuntimeStatus")
      .def(py::init<>())
      FIELD(RuntimeStatus, now_us)
      FIELD(RuntimeStatus, last_tick_us)
      FIELD(RuntimeStatus, last_acceleration_us)
      FIELD(RuntimeStatus, last_rc_us)
      FIELD(RuntimeStatus, faults)
      FIELD(RuntimeStatus, armed)
      FIELD(RuntimeStatus, arm_interlock_satisfied)
      FIELD(RuntimeStatus, output)
      FIELD(RuntimeStatus, controller);
  py::class_<ControllerConfig>(m, "ControllerConfig")
      .def(py::init<>())
      FIELD(ControllerConfig, sensor_radius_m)
      FIELD(ControllerConfig, sensor_angle_rad)
      FIELD(ControllerConfig, spin_direction)
      FIELD(ControllerConfig, minimum_phase_spin_rad_s)
      FIELD(ControllerConfig, radial_accel_floor_mps2)
      FIELD(ControllerConfig, radial_accel_filter_hz)
      FIELD(ControllerConfig, maximum_spin_rad_s)
      FIELD(ControllerConfig, spin_kp)
      FIELD(ControllerConfig, spin_ki)
      FIELD(ControllerConfig, spin_integrator_limit)
      FIELD(ControllerConfig, command_headroom)
      FIELD(ControllerConfig, translation_gain)
      FIELD(ControllerConfig, translation_phase_offset_rad);
  py::class_<RuntimeConfig>(m, "RuntimeConfig")
      .def(py::init<>())
      FIELD(RuntimeConfig, controller)
      FIELD(RuntimeConfig, acceleration_timeout_us)
      FIELD(RuntimeConfig, rc_timeout_us)
      FIELD(RuntimeConfig, maximum_tick_interval_us)
      FIELD(RuntimeConfig, control_period_us)
      FIELD(RuntimeConfig, arm_spin_max)
      FIELD(RuntimeConfig, arm_confirm_ticks);
  py::class_<PhysicalConfig>(m, "PhysicalConfig")
      .def(py::init<>())
      FIELD(PhysicalConfig, mass_kg)
      FIELD(PhysicalConfig, body_radius_m)
      FIELD(PhysicalConfig, moment_of_inertia_kg_m2)
      FIELD(PhysicalConfig, wheel_radius_m)
      FIELD(PhysicalConfig, wheel_offset_m)
      FIELD(PhysicalConfig, motor_torque_constant_nm_per_a)
      FIELD(PhysicalConfig, motor_back_emf_v_per_rad_s)
      FIELD(PhysicalConfig, motor_resistance_ohm)
      FIELD(PhysicalConfig, gear_ratio)
      FIELD(PhysicalConfig, drivetrain_efficiency)
      FIELD(PhysicalConfig, motor_inertia_kg_m2)
      FIELD(PhysicalConfig, motor_current_limit_a)
      FIELD(PhysicalConfig, motor_time_constant_s)
      FIELD(PhysicalConfig, tire_friction_coefficient)
      FIELD(PhysicalConfig, tire_longitudinal_stiffness_n_per_mps)
      FIELD(PhysicalConfig, linear_drag_n_per_mps)
      FIELD(PhysicalConfig, angular_drag_nm_per_rad_s)
      FIELD(PhysicalConfig, battery_voltage);
  py::class_<SensorModelConfig>(m, "SensorModelConfig")
      .def(py::init<>())
      FIELD(SensorModelConfig, radius_m)
      FIELD(SensorModelConfig, angle_rad)
      FIELD(SensorModelConfig, max_acceleration_mps2)
      FIELD(SensorModelConfig, noise_stddev_mps2)
      FIELD(SensorModelConfig, bias_x_mps2)
      FIELD(SensorModelConfig, bias_y_mps2)
      FIELD(SensorModelConfig, sample_period_us)
      FIELD(SensorModelConfig, latency_us);
  py::class_<SimulationConfig>(m, "SimulationConfig")
      .def(py::init<>())
      FIELD(SimulationConfig, physical)
      FIELD(SimulationConfig, sensor)
      FIELD(SimulationConfig, firmware)
      FIELD(SimulationConfig, physics_tick_us)
      FIELD(SimulationConfig, command_latency_us)
      FIELD(SimulationConfig, actuator_latency_us)
      FIELD(SimulationConfig, scenario_seed);
  py::class_<ResetState>(m, "ResetState")
      .def(py::init<>())
      FIELD(ResetState, x_m)
      FIELD(ResetState, y_m)
      FIELD(ResetState, heading_rad)
      FIELD(ResetState, vx_mps)
      FIELD(ResetState, vy_mps)
      FIELD(ResetState, spin_rad_s)
      FIELD(ResetState, estimated_phase_rad);
  py::class_<UserCommand>(m, "UserCommand")
      .def(py::init<>())
      FIELD(UserCommand, spin)
      FIELD(UserCommand, translate_x)
      FIELD(UserCommand, translate_y)
      FIELD(UserCommand, arm)
      FIELD(UserCommand, reset_phase);
  py::class_<WheelTelemetry>(m, "WheelTelemetry")
      .def(py::init<>())
      FIELD(WheelTelemetry, command)
      FIELD(WheelTelemetry, motor_current_a)
      FIELD(WheelTelemetry, wheel_speed_rad_s)
      FIELD(WheelTelemetry, slip_mps)
      FIELD(WheelTelemetry, requested_force_n)
      FIELD(WheelTelemetry, applied_force_n)
      FIELD(WheelTelemetry, normal_force_n)
      FIELD(WheelTelemetry, traction_limited);
  py::class_<Snapshot>(m, "Snapshot")
      .def(py::init<>())
      FIELD(Snapshot, time_us)
      FIELD(Snapshot, x_m)
      FIELD(Snapshot, y_m)
      FIELD(Snapshot, heading_rad)
      FIELD(Snapshot, vx_mps)
      FIELD(Snapshot, vy_mps)
      FIELD(Snapshot, spin_rad_s)
      FIELD(Snapshot, sensed_acceleration)
      FIELD(Snapshot, firmware)
      FIELD(Snapshot, wheel_a)
      FIELD(Snapshot, wheel_b);

  py::class_<Simulator>(m, "Simulator")
      .def(py::init<const SimulationConfig&>(), py::arg("config") = SimulationConfig{})
      .def_static("validate", &Simulator::validate)
      .def_property_readonly("config", &Simulator::config,
                             py::return_value_policy::copy)
      .def("reset", [](Simulator& self, const ResetState& state) { self.reset(state); },
           py::arg("state") = ResetState{})
      .def("set_command", &Simulator::set_command)
      .def("advance_ticks", &Simulator::advance_ticks, py::call_guard<py::gil_scoped_release>())
      .def("advance_for", &Simulator::advance_for, py::call_guard<py::gil_scoped_release>())
      .def("snapshot", &Simulator::snapshot);
}

#undef FIELD
