from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QAction, QCloseEvent
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QFileDialog, QFormLayout, QFrame, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSlider, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

try:
    import melty_sim as native
except ImportError as exc:  # pragma: no cover - exercised by launch failure
    raise SystemExit("melty_sim was not found. Build it and set PYTHONPATH=build/python") from exc

from . import config_codec
from .widgets import ArenaWidget, TelemetryPlot
from .worker import SimulationWorker


FAULTS = (
    ("INVALID CONFIGURATION", native.Fault.INVALID_CONFIGURATION),
    ("INVALID ACCELERATION", native.Fault.INVALID_ACCELERATION),
    ("ACCELERATION SATURATED", native.Fault.ACCELERATION_SATURATED),
    ("STALE ACCELERATION", native.Fault.STALE_ACCELERATION),
    ("INVALID RC", native.Fault.INVALID_RC),
    ("STALE RC", native.Fault.STALE_RC),
    ("CONTROL DEADLINE", native.Fault.CONTROL_DEADLINE),
    ("PHASE INVALID", native.Fault.PHASE_INVALID),
    ("HAL ERROR", native.Fault.HAL_ERROR),
)

LABELS = {
    "mass_kg": "Mass [kg]", "body_radius_m": "Body radius [m]",
    "moment_of_inertia_kg_m2": "Moment of inertia [kg·m²]",
    "wheel_radius_m": "Wheel radius [m]", "wheel_offset_m": "Wheel offset [m]",
    "motor_torque_constant_nm_per_a": "Motor Kt [N·m/A]",
    "motor_back_emf_v_per_rad_s": "Motor Ke [V/(rad/s)]",
    "motor_resistance_ohm": "Motor resistance [Ω]", "gear_ratio": "Gear ratio",
    "drivetrain_efficiency": "Drivetrain efficiency",
    "motor_inertia_kg_m2": "Motor inertia [kg·m²]",
    "motor_current_limit_a": "Current limit [A]", "motor_time_constant_s": "Motor time constant [s]",
    "tire_friction_coefficient": "Tire friction coefficient",
    "tire_longitudinal_stiffness_n_per_mps": "Tire stiffness [N/(m/s)]",
    "linear_drag_n_per_mps": "Linear drag [N/(m/s)]",
    "angular_drag_nm_per_rad_s": "Angular drag [N·m/(rad/s)]", "battery_voltage": "Battery [V]",
    "radius_m": "Physical sensor radius [m]", "angle_rad": "Physical sensor angle [rad]",
    "max_acceleration_mps2": "Sensor limit [m/s²]", "noise_stddev_mps2": "Noise σ [m/s²]",
    "bias_x_mps2": "X bias [m/s²]", "bias_y_mps2": "Y bias [m/s²]",
    "sample_period_us": "Sensor period [µs]", "latency_us": "Sensor latency [µs]",
    "sensor_radius_m": "Assumed sensor radius [m]", "sensor_angle_rad": "Assumed sensor angle [rad]",
    "spin_direction": "Spin direction (+1/-1)", "minimum_phase_spin_rad_s": "Min phase spin [rad/s]",
    "radial_accel_floor_mps2": "Radial accel floor [m/s²]", "radial_accel_filter_hz": "Accel filter [Hz]",
    "maximum_spin_rad_s": "Maximum spin [rad/s]", "spin_kp": "Spin Kp", "spin_ki": "Spin Ki",
    "spin_integrator_limit": "Integrator limit", "command_headroom": "Command headroom",
    "translation_gain": "Translation gain", "translation_phase_offset_rad": "Translation phase offset [rad]",
    "acceleration_timeout_us": "Accel timeout [µs]",
    "rc_timeout_us": "RC timeout [µs]", "maximum_tick_interval_us": "Deadline interval [µs]",
    "control_period_us": "Control period [µs]", "arm_spin_max": "Arm spin maximum",
    "arm_confirm_ticks": "Arm confirmation ticks", "physics_tick_us": "Physics tick [µs]",
    "command_latency_us": "Receiver latency [µs]", "actuator_latency_us": "Actuator latency [µs]",
    "scenario_seed": "Scenario seed",
    "x_m": "Initial X [m]", "y_m": "Initial Y [m]", "heading_rad": "Initial heading [rad]",
    "vx_mps": "Initial X velocity [m/s]", "vy_mps": "Initial Y velocity [m/s]",
    "spin_rad_s": "Initial spin [rad/s]", "estimated_phase_rad": "Initial estimated phase [rad]",
}


class MainWindow(QMainWindow):
    run_requested = Signal(bool)
    step_requested = Signal()
    reset_requested = Signal()
    command_requested = Signal(object)
    apply_requested = Signal(object, object)
    shutdown_requested = Signal()
    recording_boundary_requested = Signal()

    def __init__(self):
        super().__init__()
        self.setObjectName("main_window")
        self.setWindowTitle("Meltybrain Engineering Simulator")
        self.resize(1360, 820)
        self.config = native.SimulationConfig()
        self.initial_state = native.ResetState()
        self.command = native.UserCommand()
        self.snapshot = None
        self.editors = {}
        self._epoch_events = []
        self._epoch_snapshots = []
        self._recording_document = None
        self._recording_started_time_us = None
        self.record_file = self.record_writer = self.record_path = None
        self.worker_timer_destroyed = False
        self._build_ui()
        self.arena.set_config(self.config)
        self._populate_editors(config_codec.to_document(self.config, self.initial_state))
        self._start_worker()

    def _start_worker(self):
        self.thread = QThread(self)
        self.thread.setObjectName("simulation_thread")
        self.worker = SimulationWorker(native, self.config, self.initial_state)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.start)
        self.run_requested.connect(self.worker.set_running)
        self.step_requested.connect(self.worker.step)
        self.reset_requested.connect(self.worker.reset)
        self.command_requested.connect(self.worker.set_command)
        self.apply_requested.connect(self.worker.apply_config)
        self.shutdown_requested.connect(self.worker.shutdown)
        self.recording_boundary_requested.connect(self.worker.capture_recording_boundary)
        self.worker.snapshot_ready.connect(self._snapshot)
        self.worker.running_changed.connect(self._running_changed)
        self.worker.config_applied.connect(self._config_applied)
        self.worker.command_accepted.connect(self._command_accepted)
        self.worker.epoch_ending.connect(self._epoch_ending)
        self.worker.epoch_reset.connect(self._epoch_reset)
        self.worker.recording_boundary.connect(self._recording_boundary)
        self.worker.error.connect(self._show_error)
        self.worker.timer_destroyed.connect(self._worker_timer_destroyed)
        self.worker.stopped.connect(self.worker.deleteLater)
        self.worker.stopped.connect(self.thread.quit, Qt.ConnectionType.DirectConnection)
        self.thread.start()

    def _worker_timer_destroyed(self):
        self.worker_timer_destroyed = True

    def _build_ui(self):
        toolbar = self.addToolBar("Simulation")
        toolbar.setMovable(False)
        self.start_button = QPushButton("▶  START"); self.start_button.setObjectName("start_button")
        self.start_button.setCheckable(True); self.start_button.clicked.connect(self._toggle_run)
        self.step_button = QPushButton("STEP"); self.step_button.setObjectName("step_button")
        self.step_button.clicked.connect(self.step_requested)
        reset = QPushButton("RESET"); reset.setObjectName("reset_button"); reset.clicked.connect(self._reset)
        for widget in (self.start_button, self.step_button, reset): toolbar.addWidget(widget)
        toolbar.addSeparator()
        self.arm_button = QPushButton("DISARMED"); self.arm_button.setObjectName("arm_button")
        self.arm_button.setCheckable(True); self.arm_button.toggled.connect(self._command_changed)
        toolbar.addWidget(self.arm_button)
        phase = QPushButton("ZERO PHASE"); phase.clicked.connect(self._zero_phase); toolbar.addWidget(phase)
        toolbar.addSeparator()
        self.time_label = QLabel("  t = 0.000 s  "); toolbar.addWidget(self.time_label)
        self.state_label = QLabel("SIM PAUSED"); self.state_label.setObjectName("state_badge"); toolbar.addWidget(self.state_label)

        splitter = QSplitter(); self.setCentralWidget(splitter)
        arena_frame = QFrame(); arena_layout = QVBoxLayout(arena_frame)
        title = QLabel("ARENA / PHYSICAL TRUTH"); title.setObjectName("section_title"); arena_layout.addWidget(title)
        self.arena = ArenaWidget(); arena_layout.addWidget(self.arena, 1); splitter.addWidget(arena_frame)
        self.tabs = QTabWidget(); splitter.addWidget(self.tabs); splitter.setSizes([780, 580])
        self.tabs.addTab(self._controls_tab(), "Controls")
        self.tabs.addTab(self._telemetry_tab(), "Telemetry")
        self._menu()

    def _controls_tab(self):
        body = QWidget(); layout = QVBoxLayout(body)
        live = QFrame(); form = QFormLayout(live)
        self.spin_slider = QSlider(Qt.Orientation.Horizontal); self.spin_slider.setRange(0, 1000)
        self.spin_slider.valueChanged.connect(self._command_changed); form.addRow("Spin demand", self.spin_slider)
        self.direction_slider = QSlider(Qt.Orientation.Horizontal); self.direction_slider.setRange(-1800, 1800)
        self.direction_slider.valueChanged.connect(self._command_changed); form.addRow("Translate direction [°]", self.direction_slider)
        self.strength_slider = QSlider(Qt.Orientation.Horizontal); self.strength_slider.setRange(0, 1000)
        self.strength_slider.valueChanged.connect(self._command_changed); form.addRow("Translate strength", self.strength_slider)
        self.demand_label = QLabel("spin 0.000 · translate 0.000 @ 0.0°"); form.addRow("Live command", self.demand_label)
        layout.addWidget(live)
        config_tabs = QTabWidget()
        config_tabs.addTab(self._field_page("physical", config_codec.PHYSICAL_FIELDS), "Physical robot")
        config_tabs.addTab(self._field_page("sensor", config_codec.SENSOR_FIELDS), "Physical sensor")
        config_tabs.addTab(self._field_page("firmware.controller", config_codec.CONTROLLER_FIELDS), "Firmware assumptions")
        config_tabs.addTab(self._field_page("firmware", config_codec.RUNTIME_FIELDS), "Firmware runtime")
        config_tabs.addTab(self._field_page("config", config_codec.SIM_FIELDS), "Simulation")
        config_tabs.addTab(self._field_page("initial_state", config_codec.RESET_FIELDS), "Initial state")
        layout.addWidget(config_tabs, 1)
        buttons = QHBoxLayout()
        self.apply_button = QPushButton("APPLY && RESET"); self.apply_button.setObjectName("apply_button")
        self.apply_button.clicked.connect(self._apply_editors); buttons.addWidget(self.apply_button)
        defaults = QPushButton("RESTORE DEFAULTS"); defaults.clicked.connect(self._restore_defaults); buttons.addWidget(defaults)
        layout.addLayout(buttons)
        note = QLabel("Physical truth and firmware assumptions are independent. Structural changes validate atomically and reset simulation time.")
        note.setWordWrap(True); note.setObjectName("hint"); layout.addWidget(note)
        return body

    def _field_page(self, prefix, fields):
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        page = QWidget(); form = QFormLayout(page)
        for name in fields:
            edit = QLineEdit(); edit.setObjectName("field_" + prefix.replace(".", "_") + "_" + name)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight)
            self.editors[f"{prefix}.{name}"] = edit
            form.addRow(LABELS.get(name, name), edit)
        scroll.setWidget(page)
        return scroll

    def _telemetry_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        self.telemetry = TelemetryPlot(); layout.addWidget(self.telemetry)
        grid = QFormLayout()
        self.actual_spin = QLabel("0.0 rad/s"); self.estimated_spin = QLabel("0.0 rad/s")
        self.phase = QLabel("0.0 rad"); self.acceleration = QLabel("0.0, 0.0 m/s²")
        self.wheels = QLabel("A 0.000 / B 0.000"); self.forces = QLabel("A 0.00 / B 0.00 N")
        self.faults = QLabel("NONE"); self.faults.setObjectName("faults")
        self.armed_status = QLabel("DISARMED")
        for label, widget in (("Actual spin", self.actual_spin), ("Estimated spin", self.estimated_spin),
                              ("Estimated phase", self.phase), ("Measured acceleration", self.acceleration),
                              ("Motor demand", self.wheels), ("Applied wheel force", self.forces),
                              ("Runtime", self.armed_status), ("Faults", self.faults)): grid.addRow(label, widget)
        layout.addLayout(grid)
        self.record_button = QPushButton("START RECORDING"); self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self._toggle_recording); layout.addWidget(self.record_button)
        layout.addStretch(); return page

    def _menu(self):
        menu = self.menuBar().addMenu("Preset")
        load = QAction("Load…", self); load.triggered.connect(self._load_preset); menu.addAction(load)
        save = QAction("Save…", self); save.triggered.connect(self._save_preset); menu.addAction(save)

    def _populate_editors(self, document):
        c, s = document["config"], document["initial_state"]
        groups = {"physical": c["physical"], "sensor": c["sensor"],
                  "firmware.controller": c["firmware"]["controller"],
                  "firmware": c["firmware"], "config": c, "initial_state": s}
        for path, editor in self.editors.items():
            prefix, name = path.rsplit(".", 1); editor.setText(str(groups[prefix][name]))

    def _document_from_editors(self):
        doc = config_codec.to_document(self.config, self.initial_state)
        c, s = doc["config"], doc["initial_state"]
        groups = {"physical": c["physical"], "sensor": c["sensor"],
                  "firmware.controller": c["firmware"]["controller"],
                  "firmware": c["firmware"], "config": c, "initial_state": s}
        for path, editor in self.editors.items():
            prefix, name = path.rsplit(".", 1)
            text = editor.text().strip()
            groups[prefix][name] = int(text) if name in config_codec.INTEGER_FIELDS else float(text)
        return doc

    def apply_document(self, document):
        config, state = config_codec.from_document(native, document)
        self.apply_requested.emit(config, state)

    def _apply_editors(self):
        try: self.apply_document(self._document_from_editors())
        except (ValueError, OverflowError) as exc: self._show_error(str(exc))

    def _config_applied(self, config, state):
        self.config, self.initial_state = config, state
        self._epoch_events.clear(); self._epoch_snapshots.clear()
        self.arm_button.blockSignals(True); self.arm_button.setChecked(False)
        self.arm_button.setText("DISARMED"); self.arm_button.blockSignals(False)
        self.arena.set_config(config); self.telemetry.clear()
        self._populate_editors(config_codec.to_document(config, state))
        self.statusBar().showMessage("Configuration applied; simulation reset", 4000)

    def _restore_defaults(self):
        self._populate_editors(config_codec.to_document(native.SimulationConfig(), native.ResetState()))

    def _toggle_run(self, checked): self.run_requested.emit(checked)
    def _running_changed(self, running):
        self.start_button.blockSignals(True); self.start_button.setChecked(running)
        self.start_button.setText("❚❚  PAUSE" if running else "▶  START")
        self.start_button.blockSignals(False); self.step_button.setEnabled(not running)
        self.state_label.setText("SIM RUNNING" if running else "SIM PAUSED")

    def _reset(self):
        self.arm_button.setChecked(False); self.telemetry.clear(); self.reset_requested.emit()

    def _command_changed(self, *_):
        command = native.UserCommand()
        command.spin = self.spin_slider.value() / 1000.0
        strength = self.strength_slider.value() / 1000.0
        angle = math.radians(self.direction_slider.value() / 10.0)
        command.translate_x = strength * math.cos(angle)
        command.translate_y = strength * math.sin(angle)
        command.arm = self.arm_button.isChecked()
        self.command = command
        self.arm_button.setText("ARM REQUESTED" if command.arm else "DISARMED")
        self.demand_label.setText(f"spin {command.spin:.3f} · translate {strength:.3f} @ {math.degrees(angle):+.1f}°")
        self.command_requested.emit(command)

    def _zero_phase(self):
        command = self._copy_command(reset_phase=True)
        self.command = command; self.command_requested.emit(command)
        QTimer.singleShot(50, self._clear_phase_reset)
    def _clear_phase_reset(self):
        command = self._copy_command(reset_phase=False)
        self.command = command; self.command_requested.emit(command)

    def _copy_command(self, reset_phase=False):
        command = native.UserCommand()
        for name in ("spin", "translate_x", "translate_y", "arm"):
            setattr(command, name, getattr(self.command, name))
        command.reset_phase = reset_phase
        return command

    def _snapshot(self, s):
        self.snapshot = s; self._epoch_snapshots.append(s)
        self.arena.set_snapshot(s); self.telemetry.add_snapshot(s)
        self.time_label.setText(f"  t = {s.time_us / 1e6:.3f} s  ")
        self.actual_spin.setText(f"{s.spin_rad_s:+.2f} rad/s")
        self.estimated_spin.setText(f"{s.firmware.controller.spin_rad_s:+.2f} rad/s")
        valid = "VALID" if s.firmware.controller.phase_valid else "INVALID"
        self.phase.setText(f"{s.firmware.controller.phase_rad:+.3f} rad · {valid}")
        sat = " · SATURATED" if s.sensed_acceleration.saturated else ""
        self.acceleration.setText(f"{s.sensed_acceleration.x_mps2:+.1f}, {s.sensed_acceleration.y_mps2:+.1f} m/s²{sat}")
        self.wheels.setText(f"A {s.firmware.output.wheel_a:.3f} / B {s.firmware.output.wheel_b:.3f}")
        self.forces.setText(f"A {s.wheel_a.applied_force_n:+.2f} / B {s.wheel_b.applied_force_n:+.2f} N")
        limited = s.wheel_a.traction_limited or s.wheel_b.traction_limited
        self.armed_status.setText(("ARMED" if s.firmware.armed else "DISARMED") + (" · TRACTION LIMITED" if limited else ""))
        mask = int(s.firmware.faults); active = [name for name, fault in FAULTS if mask & int(fault)]
        self.faults.setText(" · ".join(active) if active else "NONE")
        self.faults.setProperty("active", bool(active)); self.faults.style().polish(self.faults)
        if self.record_writer:
            self._write_record_snapshot(s)

    def _command_accepted(self, event):
        self._epoch_events.append(dict(event))

    def _epoch_ending(self, final_snapshot):
        if self.record_writer:
            self._write_record_snapshot(final_snapshot)
            self._finish_recording(final_snapshot)

    def _epoch_reset(self):
        self._epoch_events.clear(); self._epoch_snapshots.clear()
        self.telemetry.clear(); self.arena.trail.clear()

    def _toggle_recording(self, checked):
        if checked:
            path, _ = QFileDialog.getSaveFileName(self, "Record simulation", "run.csv", "CSV (*.csv)")
            if not path:
                self.record_button.setChecked(False); return
            self._start_recording(Path(path))
        else:
            self.record_button.setEnabled(False)
            self.recording_boundary_requested.emit()

    def _recording_boundary(self, snapshot):
        self._write_record_snapshot(snapshot)
        self._finish_recording(snapshot)
        self.record_button.setEnabled(True)

    def _start_recording(self, path):
        try:
            self.record_path = Path(path)
            self.record_file = self.record_path.open("w", newline="", encoding="utf-8")
            self.record_writer = csv.writer(self.record_file)
            self.record_writer.writerow(["time_us", "x_m", "y_m", "heading_rad", "vx_mps", "vy_mps",
                                         "spin_rad_s", "estimated_phase_rad", "estimated_spin_rad_s",
                                         "wheel_a_command", "wheel_b_command", "fault_mask", "armed",
                                         "wheel_a_force_n", "wheel_b_force_n"])
            self._recording_document = config_codec.to_document(self.config, self.initial_state)
            self._recording_started_time_us = self.snapshot.time_us if self.snapshot else 0
            for snapshot in self._epoch_snapshots:
                self._write_record_snapshot(snapshot)
            self.record_button.setText("STOP RECORDING")
        except OSError as exc:
            self._abort_recording_file()
            self._show_error(f"could not start recording: {exc}")

    def _write_record_snapshot(self, s):
        if not self.record_writer:
            return
        try:
            self.record_writer.writerow([s.time_us, s.x_m, s.y_m, s.heading_rad, s.vx_mps, s.vy_mps,
                                         s.spin_rad_s, s.firmware.controller.phase_rad,
                                         s.firmware.controller.spin_rad_s, s.firmware.output.wheel_a,
                                         s.firmware.output.wheel_b, int(s.firmware.faults), int(s.firmware.armed),
                                         s.wheel_a.applied_force_n, s.wheel_b.applied_force_n])
        except (OSError, ValueError) as exc:
            self._abort_recording_file()
            self._show_error(f"recording write failed: {exc}")

    def _finish_recording(self, final_snapshot=None):
        if self.record_file:
            final_snapshot = final_snapshot or self.snapshot
            metadata = self._recording_document
            metadata["schema"] = config_codec.RECORDING_SCHEMA
            metadata["recorded_at"] = datetime.now(timezone.utc).isoformat()
            metadata["recording_started_time_us"] = self._recording_started_time_us
            metadata["final_time_us"] = final_snapshot.time_us if final_snapshot else 0
            metadata["simulated_duration_us"] = metadata["final_time_us"]
            metadata["final_state"] = self._snapshot_state(final_snapshot) if final_snapshot else None
            metadata["command_events"] = list(self._epoch_events)
            try:
                self.record_file.close()
                self.record_path.with_suffix(".json").write_text(
                    json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            except OSError as exc:
                self._show_error(f"could not finalize recording: {exc}")
        self.record_file = self.record_writer = None
        self._recording_document = None
        if hasattr(self, "record_button"):
            self.record_button.blockSignals(True); self.record_button.setChecked(False)
            self.record_button.setText("START RECORDING"); self.record_button.blockSignals(False)

    def _abort_recording_file(self):
        if self.record_file:
            try: self.record_file.close()
            except OSError: pass
        self.record_file = self.record_writer = None
        self._recording_document = None
        if hasattr(self, "record_button"):
            self.record_button.blockSignals(True); self.record_button.setChecked(False)
            self.record_button.setText("START RECORDING"); self.record_button.blockSignals(False)

    @staticmethod
    def _snapshot_state(snapshot):
        return {name: getattr(snapshot, name) for name in
                ("x_m", "y_m", "heading_rad", "vx_mps", "vy_mps", "spin_rad_s")}

    def _load_preset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load preset", "", "JSON (*.json)")
        if not path: return
        try:
            config, state = config_codec.load(native, path)
            self._populate_editors(config_codec.to_document(config, state))
            self.apply_requested.emit(config, state)
        except (OSError, json.JSONDecodeError, ValueError) as exc: self._show_error(str(exc))

    def _save_preset(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save preset", "robot-scenario.json", "JSON (*.json)")
        if not path: return
        try:
            config, state = config_codec.from_document(native, self._document_from_editors())
            config_codec.save(path, config, state)
        except (OSError, ValueError) as exc: self._show_error(str(exc))

    def _show_error(self, message):
        self.statusBar().showMessage("Rejected: " + message, 8000)
        if self.isVisible() and QApplication.platformName() != "offscreen":
            QMessageBox.warning(self, "Configuration rejected", message)

    def closeEvent(self, event: QCloseEvent):
        self.shutdown_requested.emit()
        if not self.thread.wait(500):
            event.ignore(); self.statusBar().showMessage("Waiting for simulation worker to stop")
            return
        QApplication.processEvents()
        self._finish_recording()
        event.accept()


STYLE = """
QWidget { background:#111925; color:#d6e0ec; font: 13px 'Inter','Helvetica Neue'; }
QMainWindow { background:#0b1018; } QToolBar { spacing:8px; padding:8px; border-bottom:1px solid #26364a; }
QPushButton { background:#24354a; border:1px solid #38516d; border-radius:4px; padding:7px 12px; font-weight:600; }
QPushButton:hover { background:#2d4661; } QPushButton:checked { background:#9c3f66; border-color:#ef5da8; }
QLineEdit { background:#0c131d; border:1px solid #2d4057; border-radius:3px; padding:5px; color:#b9e7f7; }
QTabWidget::pane { border:1px solid #26364a; } QTabBar::tab { padding:8px 12px; background:#172333; }
QTabBar::tab:selected { background:#29435e; color:#76d5f7; } QLabel#section_title { color:#76d5f7; font-weight:700; letter-spacing:1px; }
QLabel#hint { color:#7f91a8; } QLabel#faults[active="true"] { color:#ff718d; font-weight:700; }
QLabel#state_badge { color:#76d5f7; font-weight:700; padding-left:10px; } QMenuBar,QMenu { background:#111925; }
"""


def main(argv=None):
    parser = argparse.ArgumentParser(); parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = MainWindow(); window.show()
    if args.smoke:
        window.thread.finished.connect(app.quit)
        def begin_motion():
            window.arm_button.setChecked(True)
            window.start_button.click()
        def add_demand():
            window.spin_slider.setValue(550)
            window.direction_slider.setValue(350)
            window.strength_slider.setValue(500)
        def capture():
            window.grab().save("/tmp/meltybrain-ui.png")
            window.shutdown_requested.emit()
        QTimer.singleShot(150, begin_motion)
        QTimer.singleShot(400, add_demand)
        QTimer.singleShot(1500, capture)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
