"""Step-by-step derivations of the arena vectors from live snapshot values.

Each derivation recomputes the vector from the inputs the firmware or plant
actually used and checks the result against the value the native code reported,
so the overlay shows real arithmetic rather than a paraphrase of it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

VECTORS = ("wheel_a", "wheel_b", "net_force", "velocity", "steering")

TITLES = {
    "wheel_a": "Wheel A motor command",
    "wheel_b": "Wheel B motor command",
    "net_force": "Net tire drive force",
    "velocity": "Chassis velocity",
    "steering": "Steering demand → push",
}

COLORS = {
    "wheel_a": "#f5a44c",
    "wheel_b": "#8bd17c",
    "net_force": "#ef5da8",
    "velocity": "#76d5f7",
    "steering": "#ffd166",
}

CHECK_TOLERANCE = 1e-9


@dataclass
class Step:
    name: str
    formula: str
    numbers: str
    result: str
    lead: str = "= "  # shown before numbers; empty when numbers is a "where" clause


@dataclass
class Section:
    title: str
    steps: list[Step] = field(default_factory=list)
    note: str = ""
    inputs: list[tuple[str, str, str]] = field(default_factory=list)  # (symbol, value, source)


SETTINGS = "⚙ "  # names the Settings tab and field the value comes from


@dataclass
class Check:
    name: str
    recomputed: float
    reported: float

    @property
    def ok(self):
        return abs(self.recomputed - self.reported) <= CHECK_TOLERANCE * max(1.0, abs(self.reported))


@dataclass
class Derivation:
    key: str
    title: str
    subtitle: str
    sections: list[Section]
    checks: list[Check]


def _clamp(value, low, high):
    return max(low, min(value, high))


def _f(value, digits=4):
    return f"{value:+.{digits}f}"


def _deg(rad):
    return f"{math.degrees(rad):+.1f}°"


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def summary(key, s, cfg, steering=None):
    """One-line hover hint for a vector."""
    if key == "steering":
        if steering is None or steering.strength <= 0:
            return "Steering demand: none (dial at centre).\nClick for the live derivation."
        push = "—" if steering.push_rad is None else _deg(steering.push_rad)
        return (f"Steering: demand {_deg(steering.desired_rad)} world at {steering.strength:.0%}; "
                f"measured push {push}.\n"
                "Yellow ghost = where you asked to go; dotted = where the robot is actually pushing.\n"
                "Click for the live derivation.")
    if key in ("wheel_a", "wheel_b"):
        value = getattr(s.firmware.output, key)
        return (f"{TITLES[key]}: {value:.3f} of full throttle.\n"
                "Firmware output = spin common term ± translation modulation.\n"
                "Click for the live derivation.")
    if key == "net_force":
        net = s.wheel_a.applied_force_n - s.wheel_b.applied_force_n
        return (f"{TITLES[key]}: {net:+.3f} N along body +Y.\n"
                "Sum of both wheels' longitudinal tire forces (slip × stiffness, traction-limited).\n"
                "Click for the live derivation.")
    speed = math.hypot(s.vx_mps, s.vy_mps)
    return (f"{TITLES[key]}: {speed:.4f} m/s at {_deg(math.atan2(s.vy_mps, s.vx_mps))} (world frame).\n"
            "Integrated from net tire force minus linear drag.\n"
            "Click for the live derivation.")


def derive(key, s, cfg, steering=None):
    if key == "steering":
        return _steering(s, cfg, steering)
    if key in ("wheel_a", "wheel_b"):
        return _wheel_command(key, s, cfg)
    if key == "net_force":
        return _net_force(s, cfg)
    if key == "velocity":
        return _velocity(s, cfg)
    raise KeyError(key)


def _wheel_command(key, s, cfg):
    c = cfg.firmware.controller
    t = s.firmware.controller
    acc = s.consumed_acceleration
    cmd = s.consumed_command
    sign = 1 if key == "wheel_a" else -1
    letter = "A" if key == "wheel_a" else "B"
    dt = cfg.firmware.control_period_us * 1e-6
    checks = []

    # 1. Spin estimate from the accelerometer, exactly as Controller::update.
    cos_s, sin_s = math.cos(c.sensor_angle_rad), math.sin(c.sensor_angle_rad)
    body_x = cos_s * acc.x_mps2 - sin_s * acc.y_mps2
    measured = max(0.0, -body_x)
    alpha = 1.0 - math.exp(-2.0 * math.pi * c.radial_accel_filter_hz * dt)
    radial = t.radial_accel_mps2
    omega = math.sqrt(max(0.0, radial / c.sensor_radius_m))
    spin_est = c.spin_direction * omega
    checks.append(Check("estimated spin ω̂", spin_est, t.spin_rad_s))
    phase_valid = radial >= c.radial_accel_floor_mps2 and omega >= c.minimum_phase_spin_rad_s
    fa = SETTINGS + "Firmware assumptions › "
    inputs = Section("1 · Inputs", inputs=[
        ("a<sub>x</sub>, a<sub>y</sub>", f"{acc.x_mps2:+.2f}, {acc.y_mps2:+.2f} m/s²",
         f"accelerometer sample firmware read (sensor frame), stamped t = {acc.timestamp_us / 1e3:.3f} ms; "
         "modelled from the true motion, sensor placement, noise and latency in " + SETTINGS + "Physical sensor"),
        ("θ<sub>s</sub>", f"{c.sensor_angle_rad:g} rad", fa + "Assumed sensor angle"),
        ("r<sub>s</sub>", f"{c.sensor_radius_m:g} m", fa + "Assumed sensor radius"),
        ("f", f"{c.radial_accel_filter_hz:g} Hz", fa + "Accel filter"),
        ("Δt", f"{dt * 1e3:g} ms", SETTINGS + "Firmware runtime › Control period"),
        ("floor, min", f"{c.radial_accel_floor_mps2:g} m/s², {c.minimum_phase_spin_rad_s:g} rad/s",
         fa + "Radial accel floor, Min phase spin"),
        ("spin", f"{cmd.spin:.3f}", f"RC frame firmware read, stamped t = {cmd.timestamp_us / 1e3:.3f} ms ← Spin demand slider "
         "(arrives after " + SETTINGS + "Simulation › Receiver latency)"),
        ("T<sub>x</sub>, T<sub>y</sub>", f"{cmd.translate_x:+.3f}, {cmd.translate_y:+.3f}", "same RC frame ← direction dial"),
        ("ω<sub>max</sub>", f"{c.maximum_spin_rad_s:g} rad/s", fa + "Maximum spin"),
        ("K<sub>p</sub>, K<sub>i</sub>", f"{c.spin_kp:g}, {c.spin_ki:g}", fa + "Spin Kp, Spin Ki"),
        ("I limit", f"±{c.spin_integrator_limit:g}", fa + "Integrator limit"),
        ("headroom", f"{c.command_headroom:g}", fa + "Command headroom"),
        ("I", _f(t.spin_integrator), "firmware state: integrator carried between ticks"),
        ("φ̂", _deg(t.phase_rad), "firmware state: φ̂ += ω̂·Δt each tick; ZERO PHASE resets it"),
        ("dir", f"{c.spin_direction:+d}", fa + "Spin direction"),
        ("gain", f"{c.translation_gain:g}", fa + "Translation gain"),
        ("offset", _deg(c.translation_phase_offset_rad), fa + "Translation phase offset"),
        ("armed", "yes" if s.firmware.armed else "no", "firmware runtime ← ARM button, after the arm interlock"),
    ])
    estimate = Section("2 · Spin estimate (accelerometer → ω̂)", [
        Step("body-X accel", "a<sub>x,body</sub> = cos θ<sub>s</sub>·a<sub>x</sub> − sin θ<sub>s</sub>·a<sub>y</sub>",
             f"cos({c.sensor_angle_rad:.3f})·{_f(acc.x_mps2, 2)} − sin({c.sensor_angle_rad:.3f})·{_f(acc.y_mps2, 2)}",
             f"{_f(body_x, 2)} m/s²"),
        Step("measured radial", "a<sub>r</sub> = max(0, −a<sub>x,body</sub>)",
             f"max(0, {_f(-body_x, 2)})", f"{measured:.2f} m/s²"),
        Step("low-pass filter", "ā<sub>r</sub> += α·(a<sub>r</sub> − ā<sub>r</sub>),  α = 1 − e<sup>−2π·f·Δt</sup>",
             f"α = 1 − e<sup>−2π·{c.radial_accel_filter_hz:g}·{dt:g}</sup> = {alpha:.4f}",
             f"ā<sub>r</sub> = {radial:.2f} m/s²", lead="where "),
        Step("spin magnitude", "|ω̂| = √(ā<sub>r</sub> / r<sub>s</sub>)",
             f"√({radial:.2f} / {c.sensor_radius_m:g})", f"{omega:.2f} rad/s"),
        Step("phase valid?", "ā<sub>r</sub> ≥ floor  and  |ω̂| ≥ min",
             f"{radial:.1f} ≥ {c.radial_accel_floor_mps2:g}  and  {omega:.1f} ≥ {c.minimum_phase_spin_rad_s:g}",
             "yes" if phase_valid else "no — translation off"),
    ], note=f"Sample used: accelerometer reading stamped t = {acc.timestamp_us / 1e3:.3f} ms; "
            f"physical truth is {s.spin_rad_s:+.2f} rad/s.")

    if not s.firmware.armed:
        output = getattr(s.firmware.output, key)
        checks.append(Check(f"wheel {letter} output", 0.0, output))
        return Derivation(key, TITLES[key], "Firmware is disarmed, so both motor outputs are forced to 0.",
                          [inputs, estimate, Section("3 · Output", [Step("disarmed", "u = 0", "runtime not armed", "0.000")])],
                          checks)

    # 2. Spin PI controller → common throttle.
    target = cmd.spin * c.maximum_spin_rad_s
    error = target - omega
    checks.append(Check("spin error e", error, t.spin_error_rad_s))
    proportional = c.spin_kp * error
    integrator = t.spin_integrator
    max_common = 1.0 - c.command_headroom
    common = _clamp(proportional + integrator, 0.0, max_common)
    checks.append(Check("common term", common, t.common_command))
    spin_section = Section("3 · Spin controller (PI → common throttle)", [
        Step("target spin", "ω* = spin·ω<sub>max</sub>", f"{cmd.spin:.3f}·{c.maximum_spin_rad_s:g}", f"{target:.2f} rad/s"),
        Step("error", "e = ω* − |ω̂|", f"{target:.2f} − {omega:.2f}", f"{error:+.2f} rad/s"),
        Step("proportional", "P = K<sub>p</sub>·e", f"{c.spin_kp:g}·{error:+.2f}", _f(proportional)),
        Step("integral", "I += K<sub>i</sub>·e·Δt  (clamped ±limit, anti-windup)",
             f"limit ±{c.spin_integrator_limit:g}; firmware state", _f(integrator), lead="with "),
        Step("common", "c = clamp(P + I, 0, 1 − headroom)",
             f"clamp({_f(proportional)} + {_f(integrator)}, 0, {max_common:g})", f"{common:.4f}"),
    ])

    # 3. Translation modulation.
    magnitude = min(1.0, math.hypot(cmd.translate_x, cmd.translate_y))
    desired = math.atan2(cmd.translate_y, cmd.translate_x)
    phase = t.phase_rad
    force_angle = phase + c.spin_direction * 0.5 * math.pi + c.translation_phase_offset_rad
    requested = c.translation_gain * magnitude * math.cos(force_angle - desired)
    available = min(common, 1.0 - common)
    modulating = phase_valid and magnitude > 0.0
    modulation = _clamp(requested, -available, available) if modulating else 0.0
    checks.append(Check("modulation m", modulation, t.modulation_command))
    translate_section = Section("4 · Translation (phase-locked modulation)", [
        Step("demand", "|T| = min(1, √(T<sub>x</sub>² + T<sub>y</sub>²)),  ψ = atan2(T<sub>y</sub>, T<sub>x</sub>)",
             f"T = ({cmd.translate_x:+.3f}, {cmd.translate_y:+.3f})", f"|T| = {magnitude:.3f}, ψ = {_deg(desired)}"),
        Step("wheel A force angle", "φ<sub>A</sub> = φ̂ + dir·π/2 + offset",
             f"{_deg(phase)} + {c.spin_direction:+d}·90° + {_deg(c.translation_phase_offset_rad)}",
             _deg(_wrap(force_angle))),
        Step("requested", "m* = gain·|T|·cos(φ<sub>A</sub> − ψ)",
             f"{c.translation_gain:g}·{magnitude:.3f}·cos({_deg(_wrap(force_angle - desired))})", _f(requested)),
        Step("available", "room = min(c, 1 − c)", f"min({common:.4f}, {1 - common:.4f})", f"{available:.4f}"),
        Step("modulation", "m = clamp(m*, −room, room)" if modulating else "m = 0  (phase invalid or no demand)",
             f"clamp({_f(requested)}, ±{available:.4f})" if modulating else "—", _f(modulation)),
    ], note=f"Firmware phase φ̂ = {_deg(phase)}; true heading = {_deg(s.heading_rad)}. "
            f"RC frame stamped t = {cmd.timestamp_us / 1e3:.3f} ms (receiver latency).")

    raw = common + sign * modulation
    output = _clamp(raw, 0.0, 1.0)
    reported = getattr(s.firmware.output, key)
    checks.append(Check(f"wheel {letter} output", output, reported))
    op = "+" if sign > 0 else "−"
    output_section = Section("5 · Wheel output", [
        Step(f"wheel {letter}", f"u<sub>{letter}</sub> = clamp(c {op} m, 0, 1)",
             f"clamp({common:.4f} {op} {abs(modulation):.4f}, 0, 1)", f"<b>{output:.4f}</b>"),
    ])
    return Derivation(key, TITLES[key],
                      f"Firmware tick t = {s.firmware.last_tick_us / 1e3:.3f} ms · draws along body "
                      f"{'+' if sign > 0 else '−'}Y at wheel {letter}",
                      [inputs, estimate, spin_section, translate_section, output_section], checks)


def _wheel_force_steps(letter, w, p):
    stiffness = p.tire_longitudinal_stiffness_n_per_mps
    limit = p.tire_friction_coefficient * w.normal_force_n
    surface = w.wheel_speed_rad_s * p.wheel_radius_m
    ground = surface - w.slip_mps
    requested = stiffness * w.slip_mps
    scale = w.applied_force_n / w.requested_force_n if w.traction_limited and w.requested_force_n else 1.0
    steps = [
        Step(f"{letter}: surface speed", "v<sub>w</sub> = ω<sub>w</sub>·r", f"{w.wheel_speed_rad_s:.2f}·{p.wheel_radius_m:g}", f"{surface:+.4f} m/s"),
        Step(f"{letter}: slip", "s = v<sub>w</sub> − v<sub>ground</sub>", f"{surface:+.4f} − {ground:+.4f}", f"{w.slip_mps:+.4f} m/s"),
        Step(f"{letter}: requested", "F* = k·s", f"{stiffness:g}·{w.slip_mps:+.4f}", f"{requested:+.4f} N"),
        Step(f"{letter}: grip limit", "F<sub>max</sub> = μ·N", f"{p.tire_friction_coefficient:g}·{w.normal_force_n:.3f}", f"{limit:.3f} N"),
        Step(f"{letter}: applied", "F = F*·scale" + ("  (TRACTION LIMITED)" if w.traction_limited else ""),
             f"{requested:+.4f}·{scale:.4f}", f"<b>{w.applied_force_n:+.4f} N</b>"),
    ]
    return steps, Check(f"wheel {letter} requested force", requested, w.requested_force_n)


def _net_force(s, cfg):
    p = cfg.physical
    steps_a, check_a = _wheel_force_steps("A", s.wheel_a, p)
    steps_b, check_b = _wheel_force_steps("B", s.wheel_b, p)
    net = s.wheel_a.applied_force_n - s.wheel_b.applied_force_n
    torque = p.wheel_offset_m * (s.wheel_a.applied_force_n + s.wheel_b.applied_force_n)
    world_angle = s.heading_rad + math.pi / 2
    fx, fy = net * math.cos(world_angle), net * math.sin(world_angle)
    pr = SETTINGS + "Physical robot › "
    inputs = Section("1 · Inputs", inputs=[
        ("ω<sub>w,A</sub>, ω<sub>w,B</sub>", f"{s.wheel_a.wheel_speed_rad_s:.2f}, {s.wheel_b.wheel_speed_rad_s:.2f} rad/s",
         "plant motor state, driven by the wheel commands (motor Kt, Ke, resistance, battery, current limit)"),
        ("v<sub>ground</sub>", "per wheel", "chassis velocity + spin × wheel offset, along each wheel's rolling direction"),
        ("r", f"{p.wheel_radius_m:g} m", pr + "Wheel radius"),
        ("k", f"{p.tire_longitudinal_stiffness_n_per_mps:g} N/(m/s)", pr + "Tire stiffness"),
        ("μ", f"{p.tire_friction_coefficient:g}", pr + "Tire friction coefficient"),
        ("N", f"{s.wheel_a.normal_force_n:.3f} N", "½·m·g with m from " + pr + "Mass"),
        ("d", f"{p.wheel_offset_m:g} m", pr + "Wheel offset"),
        ("θ", _deg(s.heading_rad), "plant state: true heading"),
    ])
    combine = Section("4 · Combine on chassis", [
        Step("net along body Y", "F<sub>y</sub> = F<sub>A</sub> − F<sub>B</sub>  (B drives along −Y)",
             f"{s.wheel_a.applied_force_n:+.4f} − ({s.wheel_b.applied_force_n:+.4f})", f"<b>{net:+.4f} N</b>"),
        Step("in world frame", "F<sub>world</sub> = F<sub>y</sub>·(cos(θ+90°), sin(θ+90°))",
             f"θ = {_deg(s.heading_rad)}", f"({fx:+.4f}, {fy:+.4f}) N", lead="where "),
        Step("spin torque", "τ = d·(F<sub>A</sub> + F<sub>B</sub>)",
             f"{p.wheel_offset_m:g}·({s.wheel_a.applied_force_n:+.4f} + {s.wheel_b.applied_force_n:+.4f})", f"{torque:+.5f} N·m"),
    ], note="Lateral (sideways) tire scrub forces also act but are not drawn by this arrow.")
    return Derivation("net_force", TITLES["net_force"],
                      f"Plant contact model at t = {s.time_us / 1e3:.3f} ms · drawn from the chassis centre",
                      [inputs, Section("2 · Wheel A contact", steps_a), Section("3 · Wheel B contact", steps_b), combine],
                      [check_a, check_b])


def _velocity(s, cfg):
    p = cfg.physical
    speed = math.hypot(s.vx_mps, s.vy_mps)
    direction = math.atan2(s.vy_mps, s.vx_mps)
    c, sn = math.cos(s.heading_rad), math.sin(s.heading_rad)
    body_x = c * s.vx_mps + sn * s.vy_mps
    body_y = -sn * s.vx_mps + c * s.vy_mps
    drag_x, drag_y = -p.linear_drag_n_per_mps * s.vx_mps, -p.linear_drag_n_per_mps * s.vy_mps
    revolution = speed * 2 * math.pi / abs(s.spin_rad_s) if abs(s.spin_rad_s) > 1e-9 else float("inf")
    steps = [
        Step("components", "v = (v<sub>x</sub>, v<sub>y</sub>)", "plant state, world frame", f"({s.vx_mps:+.5f}, {s.vy_mps:+.5f}) m/s", lead="from "),
        Step("speed", "|v| = √(v<sub>x</sub>² + v<sub>y</sub>²)", f"√({s.vx_mps:+.5f}² + {s.vy_mps:+.5f}²)", f"<b>{speed:.5f} m/s</b>"),
        Step("direction", "β = atan2(v<sub>y</sub>, v<sub>x</sub>)", f"atan2({s.vy_mps:+.5f}, {s.vx_mps:+.5f})", f"<b>{_deg(direction)}</b>"),
        Step("in body frame", "v<sub>body</sub> = R(−θ)·v", f"θ = {_deg(s.heading_rad)}", f"({body_x:+.5f}, {body_y:+.5f}) m/s", lead="where "),
        Step("linear drag", "F<sub>drag</sub> = −c<sub>d</sub>·v", f"−{p.linear_drag_n_per_mps:g}·v", f"({drag_x:+.5f}, {drag_y:+.5f}) N"),
        Step("update rule", "v ← v + (F<sub>tires</sub> + F<sub>drag</sub>)/m·Δt", f"m = {p.mass_kg:g} kg, Δt = {cfg.physics_tick_us} µs (substepped)", "each physics tick", lead="where "),
        Step("travel per turn", "Δs = |v|·2π/|ω|", f"{speed:.5f}·2π/{abs(s.spin_rad_s):.2f}",
             "—" if math.isinf(revolution) else f"{revolution * 1000:.3f} mm"),
    ]
    return Derivation("velocity", TITLES["velocity"], f"Physical truth at t = {s.time_us / 1e3:.3f} ms",
                      [Section("1 · Inputs", inputs=[
                          ("v<sub>x</sub>, v<sub>y</sub>", f"{s.vx_mps:+.5f}, {s.vy_mps:+.5f} m/s",
                           "plant state, integrated each physics tick from tire forces and drag"),
                          ("θ, ω", f"{_deg(s.heading_rad)}, {s.spin_rad_s:+.2f} rad/s", "plant state: true heading and spin"),
                          ("m", f"{p.mass_kg:g} kg", SETTINGS + "Physical robot › Mass"),
                          ("c<sub>d</sub>", f"{p.linear_drag_n_per_mps:g} N/(m/s)", SETTINGS + "Physical robot › Linear drag"),
                          ("Δt", f"{cfg.physics_tick_us} µs", SETTINGS + "Simulation › Physics tick"),
                      ]), Section("2 · Derivation", steps)], [])


def _steering(s, cfg, st):
    if st is None:
        return Derivation("steering", TITLES["steering"], "No steering state yet.", [], [])
    w = abs(st.spin_rad_s)
    motor_lag = math.atan(w * st.motor_tau_s)
    delay_lag = w * st.latency_s
    inputs = Section("1 · Inputs", inputs=[
        ("ψ<sub>world</sub>, |T|", f"{_deg(st.desired_rad)}, {st.strength:.0%}", "direction dial (world frame: 0° = +X, 90° = +Y)"),
        ("θ", _deg(s.heading_rad), "plant truth: the robot's real heading (what a driver sees)"),
        ("φ̂", _deg(s.firmware.controller.phase_rad), "firmware state: its estimated phase"),
        ("ω", f"{s.spin_rad_s:+.2f} rad/s", "plant truth: real spin"),
        ("τ<sub>m</sub>", f"{st.motor_tau_s * 1e3:g} ms", SETTINGS + "Physical robot › Motor time constant"),
        ("t<sub>delay</sub>", f"{st.latency_s * 1e3:g} ms",
         SETTINGS + "Simulation › Actuator latency + Physical sensor › Sensor latency + Firmware runtime › Control period"),
        ("F̄<sub>drive</sub>", f"{st.push_n:.4f} N" + ("" if st.push_rad is None else f" @ {_deg(st.push_rad)}"),
         "plant: both wheels' drive force in world frame, low-passed over 0.15 s (≈ several revolutions)"),
    ])
    if not st.enabled:
        steps = [Step("sent", "ψ<sub>fw</sub> = ψ<sub>world</sub>", "world-aligned steering is off", _deg(st.sent_rad))]
    else:
        steps = [
            Step("phase drift", "δ = wrap(θ − φ̂)", f"{_deg(s.heading_rad)} − {_deg(s.firmware.controller.phase_rad)}", _deg(st.drift_rad)),
            Step("motor lag", "λ<sub>m</sub> = atan(|ω|·τ<sub>m</sub>)", f"atan({w:.1f}·{st.motor_tau_s:g})", _deg(motor_lag)),
            Step("delay lag", "λ<sub>d</sub> = |ω|·t<sub>delay</sub>", f"{w:.1f}·{st.latency_s:g}", _deg(delay_lag)),
            Step("trim", "τ̂ += k·(ψ<sub>world</sub> − push)·Δt", f"k = 1.5/s, " + ("updating" if st.trimming else "held (needs armed, valid phase, demand, push)"), _deg(st.trim_rad)),
            Step("sent to firmware", "ψ<sub>fw</sub> = ψ<sub>world</sub> − δ − (λ<sub>m</sub> + λ<sub>d</sub>) + τ̂",
                 f"{_deg(st.desired_rad)} − {_deg(st.drift_rad)} − {_deg(st.lag_rad)} + {_deg(st.trim_rad)}", f"<b>{_deg(st.sent_rad)}</b>"),
        ]
    measured = []
    if st.push_rad is not None:
        measured = [
            Step("push direction", "atan2(F̄<sub>y</sub>, F̄<sub>x</sub>)", f"|F̄| = {st.push_n:.4f} N", f"<b>{_deg(st.push_rad)}</b>", lead="with "),
            Step("alignment error", "ψ<sub>world</sub> − push", f"{_deg(st.desired_rad)} − {_deg(st.push_rad)}", f"<b>{_deg(st.error_rad)}</b>"),
        ]
    return Derivation("steering", TITLES["steering"],
                      "Yellow ghost = your world demand on the robot · dotted = measured push",
                      [inputs, Section("2 · World → firmware frame", steps),
                       Section("3 · Measured result", measured or [Step("push", "—", "no measurable push yet", "—", lead="")],
                               note="Drift and lag are feed-forward from simulation truth; the trim learns what the lag model misses. "
                                    "A real robot has no truth: drivers trim by eye or with a heading LED.")], [])


def to_html(d):
    color = COLORS[d.key]
    parts = [
        f"<div style='font-size:15px; font-weight:700; color:{color}'>{d.title}</div>",
        f"<div style='color:#8a9bb2; font-size:11px'>{d.subtitle}</div>",
    ]
    for section in d.sections:
        parts.append(f"<div style='color:#76d5f7; font-weight:700; margin-top:8px'>{section.title}</div>")
        if section.inputs:
            parts.append("<table cellspacing='0' cellpadding='1' style='font-size:12px'>")
            for symbol, value, source in section.inputs:
                parts.append(
                    f"<tr><td style='color:#d6e0ec; padding-right:8px'>{symbol}</td>"
                    f"<td style='color:{color}; padding-right:8px; white-space:nowrap'>{value}</td>"
                    f"<td style='color:#7f91a8'>{source}</td></tr>")
            parts.append("</table>")
        parts.append("<table cellspacing='0' cellpadding='1' style='font-size:12px'>")
        for step in section.steps:
            parts.append(
                "<tr>"
                f"<td style='color:#8a9bb2; padding-right:8px' rowspan='2'>{step.name}</td>"
                f"<td style='color:#d6e0ec'>{step.formula}</td></tr>"
                f"<tr><td style='color:#9fb3c8; padding-bottom:4px'>{step.lead}{step.numbers} "
                f"<span style='color:{color}'>→ {step.result}</span></td></tr>")
        parts.append("</table>")
        if section.note:
            parts.append(f"<div style='color:#6f8199; font-size:11px'>{section.note}</div>")
    if d.checks:
        ok = all(check.ok for check in d.checks)
        badge = ("<span style='color:#8bd17c'>✓ recomputed values match the native code</span>" if ok
                 else "<span style='color:#ff718d'>✗ mismatch with native code</span>")
        parts.append(f"<div style='margin-top:8px; font-size:11px'>{badge}</div>")
        if not ok:
            for check in d.checks:
                if not check.ok:
                    parts.append(f"<div style='color:#ff718d; font-size:11px'>{check.name}: "
                                 f"{check.recomputed:.6g} vs {check.reported:.6g}</div>")
    parts.append("<div style='color:#5d6f86; font-size:11px; margin-top:6px'>Click empty space or press Esc to close. "
                 "Pause or slow the sim to read along.</div>")
    return "".join(parts)
