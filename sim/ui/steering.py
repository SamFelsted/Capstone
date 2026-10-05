"""World-aligned steering: the live controls act like a driver watching the robot.

Firmware steers in its own phase frame, which differs from the world by the
phase-estimate drift (true heading minus estimated phase) and by the
speed-dependent lag between commanding a push and the tires delivering it
(motor time constant plus sensor, control and actuator delays). This assist
rotates the dial's world direction into the firmware frame:

    sent = desired − drift − lag(ω) + trim

drift and lag are feed-forward from simulation truth; trim is a slow integral
on the measured push direction (the revolution-averaged wheel drive force) that
removes whatever the lag model misses. It is a simulator convenience standing
in for a driver's eyes; the firmware itself never sees truth.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

TRIM_GAIN_PER_S = 1.5
TRIM_LIMIT_RAD = math.pi / 2
PUSH_THRESHOLD_N = 0.005


def wrap(angle):
    return math.remainder(angle, 2 * math.pi)


@dataclass
class SteeringState:
    desired_rad: float = 0.0
    strength: float = 0.0
    sent_rad: float = 0.0
    drift_rad: float = 0.0
    lag_rad: float = 0.0
    trim_rad: float = 0.0
    push_rad: float | None = None
    push_n: float = 0.0
    error_rad: float | None = None
    enabled: bool = True
    latency_s: float = 0.0
    motor_tau_s: float = 0.0
    spin_rad_s: float = 0.0
    trimming: bool = False
    heading_at_tick_rad: float = 0.0


class WorldSteering:
    def __init__(self):
        self.enabled = True
        self.trim_rad = 0.0
        self._last_time_us = None
        self.state = SteeringState()

    def reset(self):
        self.trim_rad = 0.0
        self._last_time_us = None

    def update(self, snapshot, config, desired_rad, strength):
        """Return the firmware-frame angle to send for a world-frame demand."""
        state = SteeringState(desired_rad=desired_rad, strength=strength, enabled=self.enabled,
                              trim_rad=self.trim_rad)
        if snapshot is not None:
            omega = snapshot.spin_rad_s
            latency = (config.actuator_latency_us + config.sensor.latency_us
                       + config.firmware.control_period_us) * 1e-6
            tau = config.physical.motor_time_constant_s
            state.spin_rad_s, state.latency_s, state.motor_tau_s = omega, latency, tau
            # Compare at the same instant: the firmware phase is from its last
            # control tick, so wind the true heading back to that tick.
            since_tick = max(0, snapshot.time_us - snapshot.firmware.now_us) * 1e-6
            state.heading_at_tick_rad = wrap(snapshot.heading_rad - omega * since_tick)
            state.drift_rad = wrap(state.heading_at_tick_rad - snapshot.firmware.controller.phase_rad)
            state.lag_rad = math.copysign(math.atan(abs(omega) * tau) + abs(omega) * latency, omega)
            fx, fy = snapshot.mean_drive_force_x_n, snapshot.mean_drive_force_y_n
            state.push_n = math.hypot(fx, fy)
            if state.push_n > PUSH_THRESHOLD_N:
                state.push_rad = math.atan2(fy, fx)
                state.error_rad = wrap(desired_rad - state.push_rad)
            dt = 0.0
            if self._last_time_us is not None and snapshot.time_us >= self._last_time_us:
                dt = min(0.1, (snapshot.time_us - self._last_time_us) * 1e-6)
            self._last_time_us = snapshot.time_us
            state.trimming = (self.enabled and strength > 0.0 and state.error_rad is not None
                              and snapshot.firmware.armed and snapshot.firmware.controller.phase_valid)
            if state.trimming:
                self.trim_rad = max(-TRIM_LIMIT_RAD, min(TRIM_LIMIT_RAD,
                                    self.trim_rad + TRIM_GAIN_PER_S * state.error_rad * dt))
                state.trim_rad = self.trim_rad
        if self.enabled and snapshot is not None:
            state.sent_rad = wrap(desired_rad - state.drift_rad - state.lag_rad + state.trim_rad)
        else:
            state.sent_rad = desired_rad
        self.state = state
        return state.sent_rad
