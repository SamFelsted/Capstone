"""Versioned JSON codec for simulator presets.

The C++ types remain the authority for defaults and validation.  This module is
deliberately explicit so a preset cannot silently omit a newly exposed field.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

SCHEMA = "meltybrain-simulator"
VERSION = 1
RECORDING_SCHEMA = "meltybrain-simulator-recording"

PHYSICAL_FIELDS = (
    "mass_kg", "body_radius_m", "moment_of_inertia_kg_m2", "wheel_radius_m",
    "wheel_offset_m", "motor_torque_constant_nm_per_a",
    "motor_back_emf_v_per_rad_s", "motor_resistance_ohm", "gear_ratio",
    "drivetrain_efficiency", "motor_inertia_kg_m2", "motor_current_limit_a",
    "motor_time_constant_s", "tire_friction_coefficient",
    "tire_longitudinal_stiffness_n_per_mps", "linear_drag_n_per_mps",
    "angular_drag_nm_per_rad_s", "battery_voltage",
)
SENSOR_FIELDS = (
    "radius_m", "angle_rad", "max_acceleration_mps2", "noise_stddev_mps2",
    "bias_x_mps2", "bias_y_mps2", "sample_period_us", "latency_us",
)
CONTROLLER_FIELDS = (
    "sensor_radius_m", "sensor_angle_rad", "spin_direction",
    "minimum_phase_spin_rad_s", "radial_accel_floor_mps2",
    "radial_accel_filter_hz", "maximum_spin_rad_s", "spin_kp", "spin_ki",
    "spin_integrator_limit", "command_headroom", "translation_gain",
    "translation_phase_offset_rad",
)
RUNTIME_FIELDS = (
    "acceleration_timeout_us", "rc_timeout_us", "maximum_tick_interval_us",
    "control_period_us", "arm_spin_max", "arm_confirm_ticks",
)
SIM_FIELDS = ("physics_tick_us", "command_latency_us", "actuator_latency_us", "scenario_seed")
RESET_FIELDS = (
    "x_m", "y_m", "heading_rad", "vx_mps", "vy_mps", "spin_rad_s",
    "estimated_phase_rad",
)
INTEGER_FIELDS = {
    "sample_period_us", "latency_us", "spin_direction",
    "acceleration_timeout_us", "rc_timeout_us", "maximum_tick_interval_us",
    "control_period_us", "arm_confirm_ticks", "physics_tick_us",
    "command_latency_us", "actuator_latency_us", "scenario_seed",
}
UINT64_MAX = (1 << 64) - 1
UINT32_MAX = (1 << 32) - 1
INTEGER_LIMITS = {
    "spin_direction": (-(1 << 31), (1 << 31) - 1),
    "arm_confirm_ticks": (0, UINT32_MAX),
    **{name: (0, UINT64_MAX) for name in INTEGER_FIELDS
       if name not in {"spin_direction", "arm_confirm_ticks"}},
}


def _values(obj: Any, names: tuple[str, ...]) -> dict[str, Any]:
    return {name: getattr(obj, name) for name in names}


def to_document(config: Any, state: Any) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "version": VERSION,
        "config": {
            "physical": _values(config.physical, PHYSICAL_FIELDS),
            "sensor": _values(config.sensor, SENSOR_FIELDS),
            "firmware": {
                **_values(config.firmware, RUNTIME_FIELDS),
                "controller": _values(config.firmware.controller, CONTROLLER_FIELDS),
            },
            **_values(config, SIM_FIELDS),
        },
        "initial_state": _values(state, RESET_FIELDS),
    }


def _require_mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return value


def _assign(target: Any, source: dict[str, Any], names: tuple[str, ...], path: str) -> None:
    missing = [name for name in names if name not in source]
    extra = sorted(set(source) - set(names))
    if missing or extra:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unknown " + ", ".join(extra))
        raise ValueError(f"{path}: {'; '.join(details)}")
    for name in names:
        raw = source[name]
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{path}.{name} must be numeric")
        if name in INTEGER_FIELDS:
            if isinstance(raw, float) and not math.isfinite(raw):
                raise ValueError(f"{path}.{name} must be finite")
            if int(raw) != raw:
                raise ValueError(f"{path}.{name} must be an integer")
            raw = int(raw)
            low, high = INTEGER_LIMITS[name]
            if not low <= raw <= high:
                raise ValueError(f"{path}.{name} must be in [{low}, {high}]")
        elif not math.isfinite(float(raw)):
            raise ValueError(f"{path}.{name} must be finite")
        try:
            setattr(target, name, raw)
        except (TypeError, OverflowError, ValueError) as exc:
            raise ValueError(f"{path}.{name}: {exc}") from exc


def from_document(module: Any, document: dict[str, Any]) -> tuple[Any, Any]:
    root = _require_mapping(document, "preset")
    if root.get("schema") != SCHEMA or root.get("version") != VERSION:
        raise ValueError(f"unsupported preset schema/version (expected {SCHEMA} v{VERSION})")
    allowed = {"schema", "version", "config", "initial_state"}
    if set(root) - allowed:
        raise ValueError("unknown preset keys: " + ", ".join(sorted(set(root) - allowed)))
    raw_config = _require_mapping(root.get("config"), "config")
    raw_state = _require_mapping(root.get("initial_state"), "initial_state")
    config = module.SimulationConfig()
    state = module.ResetState()
    physical = _require_mapping(raw_config.get("physical"), "config.physical")
    sensor = _require_mapping(raw_config.get("sensor"), "config.sensor")
    firmware = _require_mapping(raw_config.get("firmware"), "config.firmware")
    controller = _require_mapping(firmware.get("controller"), "config.firmware.controller")
    _assign(config.physical, physical, PHYSICAL_FIELDS, "config.physical")
    _assign(config.sensor, sensor, SENSOR_FIELDS, "config.sensor")
    _assign(config.firmware.controller, controller, CONTROLLER_FIELDS,
            "config.firmware.controller")
    if set(firmware) != {*RUNTIME_FIELDS, "controller"}:
        raise ValueError("config.firmware keys do not match schema")
    _assign(config.firmware, {k: firmware[k] for k in RUNTIME_FIELDS}, RUNTIME_FIELDS,
            "config.firmware")
    sim_values = {name: raw_config[name] for name in SIM_FIELDS if name in raw_config}
    expected_config_keys = {"physical", "sensor", "firmware", *SIM_FIELDS}
    if set(raw_config) != expected_config_keys:
        raise ValueError("config keys do not match schema")
    _assign(config, sim_values, SIM_FIELDS, "config")
    _assign(state, raw_state, RESET_FIELDS, "initial_state")
    errors = list(module.Simulator.validate(config))
    if errors:
        raise ValueError("; ".join(errors))
    try:
        candidate = module.Simulator(config)
        candidate.reset(state)
    except (TypeError, OverflowError, ValueError, RuntimeError) as exc:
        raise ValueError(f"initial_state: {exc}") from exc
    return config, state


def load(module: Any, path: str | Path) -> tuple[Any, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return from_document(module, json.load(handle))


def from_recording(module: Any, document: dict[str, Any]) -> tuple[Any, Any]:
    root = _require_mapping(document, "recording")
    if root.get("schema") != RECORDING_SCHEMA or root.get("version") != VERSION:
        raise ValueError(f"unsupported recording schema/version (expected {RECORDING_SCHEMA} v{VERSION})")
    return from_document(module, {
        "schema": SCHEMA,
        "version": VERSION,
        "config": root.get("config"),
        "initial_state": root.get("initial_state"),
    })


def save(path: str | Path, config: Any, state: Any) -> None:
    destination = Path(path)
    destination.write_text(json.dumps(to_document(config, state), indent=2) + "\n",
                           encoding="utf-8")
