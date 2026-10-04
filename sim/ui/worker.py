from __future__ import annotations

import time

from PySide6.QtCore import QObject, QTimer, Qt, Signal, Slot


class SimulationWorker(QObject):
    """Owns the native Simulator and runs exclusively on its QThread."""

    snapshot_ready = Signal(object)
    running_changed = Signal(bool)
    config_applied = Signal(object, object)
    command_accepted = Signal(object)
    epoch_ending = Signal(object)
    epoch_reset = Signal()
    recording_boundary = Signal(object)
    error = Signal(str)
    stopped = Signal()
    timer_destroyed = Signal()

    def __init__(self, native, config, state):
        super().__init__()
        self.native = native
        self._config = config
        self._state = state
        self._simulator = None
        self._timer = None
        self._running = False
        self._last_wall_ns = 0
        self._last_render_ns = 0
        self._accumulated_us = 0
        self._time_scale = 1.0
        self._command = native.UserCommand()

    @Slot()
    def start(self):
        try:
            self._simulator = self.native.Simulator(self._config)
            self._simulator.reset(self._state)
            self._timer = QTimer(self)
            self._timer.destroyed.connect(self.timer_destroyed)
            self._timer.setTimerType(Qt.TimerType.PreciseTimer)
            self._timer.setInterval(4)
            self._timer.timeout.connect(self._pump)
            self._timer.start()
            self._emit_snapshot()
        except Exception as exc:  # keep exceptions out of Qt's event loop
            self.error.emit(str(exc))

    @Slot()
    def shutdown(self):
        self._running = False
        if self._simulator is not None:
            try:
                self.epoch_ending.emit(self._simulator.snapshot())
            except Exception as exc:
                self.error.emit(str(exc))
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
        self._simulator = None
        self.stopped.emit()

    @Slot()
    def capture_recording_boundary(self):
        if self._simulator is None:
            return
        try:
            self.recording_boundary.emit(self._simulator.snapshot())
        except Exception as exc:
            self._fail(exc)

    @Slot()
    def request_phase_reset(self):
        """Deliver a phase-reset pulse through the normal simulated RC path."""
        if self._simulator is None:
            return
        try:
            status = self._simulator.snapshot().firmware
            if status.armed or self._command.arm:
                raise ValueError("phase reset requires the robot to be disarmed")
            # Commands are acquired only at absolute control boundaries. Hold
            # reset through the next acquisition, receiver transport, and the
            # first control boundary that can consume the delivered frame.
            now_us = int(self._simulator.snapshot().time_us)
            period_us = int(self._config.firmware.control_period_us)
            next_acquisition_us = (now_us // period_us + 1) * period_us
            delivered_us = next_acquisition_us + int(self._config.command_latency_us)
            consumption_us = ((delivered_us + period_us - 1) // period_us) * period_us
            advance_us = consumption_us - now_us
            advance_ticks = advance_us // int(self._config.physics_tick_us)
            if advance_ticks > 10_000:
                raise ValueError(
                    "synchronous phase-reset window exceeds the 10000-tick action budget; "
                    "use lower receiver delay, a larger physics timestep, or Apply & Reset "
                    "with the desired estimated phase")

            asserted = self._copy_command(self._command)
            asserted.reset_phase = True
            assertion_published = False
            action_error = None
            try:
                self._command = asserted
                self._simulator.set_command(asserted)
                assertion_published = True
                self._acknowledge_command(asserted)
                self._simulator.advance_for(advance_us)
                consumed = self._simulator.snapshot()
                blocking_fault_mask = 0
                for fault in (
                    self.native.Fault.INVALID_CONFIGURATION,
                    self.native.Fault.INVALID_ACCELERATION,
                    self.native.Fault.STALE_ACCELERATION,
                    self.native.Fault.INVALID_RC,
                    self.native.Fault.STALE_RC,
                    self.native.Fault.CONTROL_DEADLINE,
                    self.native.Fault.HAL_ERROR,
                    self.native.Fault.ACCELERATION_SATURATED,
                    self.native.Fault.CONTROLLER_NUMERIC,
                ):
                    blocking_fault_mask |= int(fault)
                if consumed.firmware.armed:
                    raise RuntimeError("phase reset did not leave the robot disarmed")
                if int(consumed.firmware.faults) & blocking_fault_mask:
                    raise RuntimeError(
                        "phase reset could not be consumed because firmware reported a blocking fault")
                if abs(consumed.firmware.controller.phase_rad) > 1.0e-6:
                    raise RuntimeError("firmware did not consume the phase reset command")
            except Exception as exc:
                action_error = exc
            finally:
                if assertion_published:
                    try:
                        cleared = self._copy_command(asserted)
                        cleared.reset_phase = False
                        self._command = cleared
                        self._simulator.set_command(cleared)
                        self._acknowledge_command(cleared)
                    except Exception as clear_exc:
                        if action_error is None:
                            action_error = clear_exc
                self._emit_snapshot()
            if action_error is not None:
                raise action_error
        except Exception as exc:
            self._fail(exc)

    @Slot(bool)
    def set_running(self, running):
        self._running = bool(running)
        self._last_wall_ns = time.monotonic_ns()
        self.running_changed.emit(self._running)

    @Slot(float)
    def set_time_scale(self, scale):
        """Simulated seconds advanced per wall-clock second."""
        self._time_scale = max(0.0, float(scale))

    @Slot()
    def step(self):
        if self._simulator is None or self._running:
            return
        try:
            self._simulator.advance_ticks(1)
            self._emit_snapshot()
        except Exception as exc:
            self._fail(exc)

    @Slot()
    def reset(self):
        if self._simulator is None:
            return
        try:
            self.epoch_ending.emit(self._simulator.snapshot())
            self._running = False
            command = self._copy_command(self._command)
            command.arm = False
            command.reset_phase = False
            self._command = command
            self._simulator.reset(self._state)
            self._simulator.set_command(command)
            self._accumulated_us = 0
            self.running_changed.emit(False)
            self.epoch_reset.emit()
            self._acknowledge_command(command)
            self._emit_snapshot()
        except Exception as exc:
            self._fail(exc)

    @Slot(object)
    def set_command(self, command):
        try:
            owned = self._copy_command(command)
            self._command = owned
            if self._simulator is not None:
                self._simulator.set_command(owned)
                self._acknowledge_command(owned)
        except Exception as exc:
            self._fail(exc)

    @Slot(object, object)
    def apply_config(self, config, state):
        """Validate and construct before replacing the live simulator."""
        try:
            errors = list(self.native.Simulator.validate(config))
            if errors:
                raise ValueError("; ".join(errors))
            candidate = self.native.Simulator(config)
            candidate.reset(state)
            command = self._copy_command(self._command)
            command.arm = False
            command.reset_phase = False
            candidate.set_command(command)
        except Exception as exc:
            self.error.emit(str(exc))
            return
        try:
            if self._simulator is not None:
                self.epoch_ending.emit(self._simulator.snapshot())
            self._running = False
            self._simulator = candidate
            self._command = command
            self._config = config
            self._state = state
            self._accumulated_us = 0
            self.running_changed.emit(False)
            self.config_applied.emit(config, state)
            self._acknowledge_command(command)
            self._emit_snapshot()
        except Exception as exc:
            self._fail(exc)

    @Slot()
    def _pump(self):
        if not self._running or self._simulator is None:
            return
        now = time.monotonic_ns()
        if not self._last_wall_ns:
            self._last_wall_ns = now
            return
        elapsed_us = max(0, (now - self._last_wall_ns) // 1000)
        self._last_wall_ns = now
        tick_us = max(1, int(self._config.physics_tick_us))
        # Preserve fractional time across pumps. Cap one second of backlog: under
        # sustained overload the simulator deliberately slows instead of freezing
        # the UI while attempting an unbounded catch-up.
        self._accumulated_us = min(
            self._accumulated_us + elapsed_us * self._time_scale, 1_000_000)
        ticks = min(200, int(self._accumulated_us // tick_us))
        try:
            if ticks:
                self._simulator.advance_ticks(ticks)
                self._accumulated_us -= ticks * tick_us
            if now - self._last_render_ns >= 33_000_000:
                self._last_render_ns = now
                self._emit_snapshot()
        except Exception as exc:
            self._fail(exc)

    def _emit_snapshot(self):
        if self._simulator is not None:
            self.snapshot_ready.emit(self._simulator.snapshot())

    def _copy_command(self, source):
        command = self.native.UserCommand()
        for name in ("spin", "translate_x", "translate_y", "arm", "reset_phase"):
            setattr(command, name, getattr(source, name))
        return command

    def _acknowledge_command(self, command):
        now_us = self._simulator.snapshot().time_us
        self.command_accepted.emit({
            "time_us": now_us,
            "spin": command.spin,
            "translate_x": command.translate_x,
            "translate_y": command.translate_y,
            "arm": command.arm,
            "reset_phase": command.reset_phase,
        })

    def _fail(self, exc):
        self._running = False
        self.running_changed.emit(False)
        self.error.emit(str(exc))
