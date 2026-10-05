from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QAction, QCloseEvent
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QDialog, QFileDialog, QFormLayout, QFrame, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMenu, QMessageBox, QPushButton, QScrollArea,
    QSizePolicy, QSlider, QSplitter, QStackedWidget, QTabWidget, QVBoxLayout, QWidget,
)

try:
    import melty_sim as native
except ImportError as exc:  # pragma: no cover - exercised by launch failure
    raise SystemExit("melty_sim was not found. Build it and set PYTHONPATH=build/python") from exc

from . import config_codec
from .flow_diagram import FlowDiagram
from .widgets import ArenaWidget, DirectionDial, TelemetryPlot
from .steering import WorldSteering
from .worker import SimulationWorker


FAULTS = (
    ("INVALID CONFIGURATION", native.Fault.INVALID_CONFIGURATION),
    ("INVALID ACCELERATION", native.Fault.INVALID_ACCELERATION),
    ("ACCELERATION SATURATED", native.Fault.ACCELERATION_SATURATED),
    ("CONTROLLER NUMERIC", native.Fault.CONTROLLER_NUMERIC),
    ("STALE ACCELERATION", native.Fault.STALE_ACCELERATION),
    ("INVALID RC", native.Fault.INVALID_RC),
    ("STALE RC", native.Fault.STALE_RC),
    ("CONTROL DEADLINE", native.Fault.CONTROL_DEADLINE),
    ("PHASE INVALID", native.Fault.PHASE_INVALID),
    ("HAL ERROR", native.Fault.HAL_ERROR),
)

PRESET_DIR = Path(__file__).resolve().parents[1] / "config"

# Minimum change in the steering-corrected direction before it is re-sent.
STEERING_RESEND_DEG = 0.5

# Spin demand is capped to this fraction of what the accelerometer can measure.
SPIN_SENSOR_MARGIN = 0.9

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
    phase_reset_requested = Signal()
    time_scale_requested = Signal(float)

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
        self.steering = WorldSteering()
        self._was_armed = False
        self._last_disarm = None
        self._epoch_events = []
        self._epoch_spool = self._epoch_spool_writer = None
        self._recording_document = None
        self._recording_started_time_us = None
        self.record_file = self.record_writer = self.record_path = None
        self.worker_timer_destroyed = False
        self._build_ui()
        self._replace_epoch_spool()
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
        self.phase_reset_requested.connect(self.worker.request_phase_reset)
        self.time_scale_requested.connect(self.worker.set_time_scale)
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
        self.thread.finished.connect(self._close_epoch_spool)
        self.thread.start()

    def _worker_timer_destroyed(self):
        self.worker_timer_destroyed = True

    def _build_ui(self):
        toolbar = self.addToolBar("Simulation")
        toolbar.setMovable(False)
        self.start_button = QPushButton("▶  START"); self.start_button.setObjectName("start_button")
        self.start_button.setCheckable(True); self.start_button.clicked.connect(self._toggle_run)
        self.start_button.setToolTip("Run or pause simulated time.")
        self.step_button = QPushButton("STEP"); self.step_button.setObjectName("step_button")
        self.step_button.clicked.connect(self.step_requested)
        self.step_button.setToolTip("Advance one physics tick while paused.")
        reset = QPushButton("RESET"); reset.setObjectName("reset_button"); reset.clicked.connect(self._reset)
        reset.setToolTip("Return to the initial state, disarm, and clear history.")
        for widget in (self.start_button, self.step_button, reset): toolbar.addWidget(widget)
        toolbar.addSeparator()
        self.arm_button = QPushButton("DISARMED"); self.arm_button.setObjectName("arm_button")
        self.arm_button.setCheckable(True); self.arm_button.toggled.connect(self._command_changed)
        toolbar.addWidget(self.arm_button)
        phase = QPushButton("ZERO PHASE"); phase.clicked.connect(self._zero_phase); toolbar.addWidget(phase)
        phase.setToolTip("While disarmed, declare the robot's current orientation as phase 0\n"
                         "(the reference the direction dial is measured from).")
        toolbar.addSeparator()
        toolbar.addWidget(QLabel("Sim speed"))
        # Logarithmic: slider value v maps to 10^(v/100)x real time, 0.001x to 4x.
        self.speed_slider = QSlider(Qt.Orientation.Horizontal); self.speed_slider.setObjectName("speed_slider")
        self.speed_slider.setRange(-300, 60); self.speed_slider.setValue(0); self.speed_slider.setFixedWidth(160)
        self.speed_slider.setToolTip("Simulated time per real second. Slow down to watch individual rotations.")
        self.speed_slider.valueChanged.connect(self._speed_changed); toolbar.addWidget(self.speed_slider)
        self.speed_label = QLabel("1.000×"); self.speed_label.setFixedWidth(56); toolbar.addWidget(self.speed_label)
        self.realtime_button = QPushButton("1×"); self.realtime_button.setObjectName("realtime_button")
        self.realtime_button.setToolTip("Snap simulation speed back to real time.")
        self.realtime_button.clicked.connect(lambda: self.speed_slider.setValue(0)); toolbar.addWidget(self.realtime_button)
        toolbar.addSeparator()
        self.time_label = QLabel("  t = 0.000 s  "); toolbar.addWidget(self.time_label)
        self.state_label = QLabel("SIM PAUSED"); self.state_label.setObjectName("state_badge"); toolbar.addWidget(self.state_label)
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)
        self.presets_button = QPushButton("PRESETS"); self.presets_button.setObjectName("presets_button")
        self.presets_button.setToolTip("Load a robot configuration, including failure-mode demonstrations.")
        toolbar.addWidget(self.presets_button)
        self.settings_button = QPushButton("⚙  SETTINGS"); self.settings_button.setObjectName("settings_button")
        self.settings_button.setToolTip("Robot, sensor, firmware and simulation configuration, and presets.")
        self.settings_button.clicked.connect(self._show_settings); toolbar.addWidget(self.settings_button)

        splitter = QSplitter(); self.setCentralWidget(splitter)
        arena_frame = QFrame(); arena_layout = QVBoxLayout(arena_frame)
        header = QHBoxLayout()
        title = QLabel("ARENA / PHYSICAL TRUTH"); title.setObjectName("section_title"); header.addWidget(title)
        header.addStretch()
        self.vectors_check = QCheckBox("Motor vectors"); self.vectors_check.setObjectName("vectors_check")
        self.vectors_check.setToolTip("Show wheel commands, net tire force and velocity.\n"
                                      "Hover a vector for a summary; click it for the live math.")
        self.vectors_check.setChecked(True); header.addWidget(self.vectors_check)
        arena_layout.addLayout(header)
        self.arena = ArenaWidget(); arena_layout.addWidget(self.arena, 1); splitter.addWidget(arena_frame)
        self.vectors_check.toggled.connect(self.arena.set_show_vectors)
        side = QScrollArea(); side.setWidgetResizable(True); side.setFrameShape(QFrame.Shape.NoFrame)
        side.setWidget(self._side_panel()); splitter.addWidget(side); splitter.setSizes([820, 540])
        self.settings_dialog = self._settings_dialog()
        self._menu()

    def _side_panel(self):
        body = QWidget(); layout = QVBoxLayout(body)
        title = QLabel("LIVE CONTROLS"); title.setObjectName("section_title"); layout.addWidget(title)
        self.preset_label = QLabel(); self.preset_label.setObjectName("preset_note")
        self.preset_label.setTextFormat(Qt.TextFormat.RichText); self.preset_label.setWordWrap(True)
        self.preset_label.hide(); layout.addWidget(self.preset_label)
        form = QFormLayout()
        self.spin_slider = QSlider(Qt.Orientation.Horizontal); self.spin_slider.setRange(0, 1000)
        self.spin_slider.setToolTip("Spin demand as a fraction of the firmware's maximum spin.\n"
                                    "Must be at or below the arm limit when arming.")
        self.spin_slider.valueChanged.connect(self._command_changed)
        spin_hint = ("How fast you ask the robot to spin, as a fraction of the firmware's maximum spin\n"
                     "(⚙ Firmware assumptions › Maximum spin). The spin controller holds that target speed.\n"
                     "Faster spin hits harder but needs more steering lag compensation, and arming requires\n"
                     "it to be at or below the arm limit (⚙ Firmware runtime › Arm spin maximum).")
        spin_label = QLabel("Spin demand"); spin_label.setToolTip(spin_hint); self.spin_slider.setToolTip(spin_hint)
        self.spin_value = QLabel("0% → 0 rad/s"); self.spin_value.setToolTip(spin_hint); self.spin_value.setMinimumWidth(110)
        self.spin_value.setTextFormat(Qt.TextFormat.RichText)
        spin_row = QHBoxLayout(); spin_row.addWidget(self.spin_slider, 1); spin_row.addWidget(self.spin_value)
        form.addRow(spin_label, spin_row)
        self.spin_limit_check = QCheckBox("Limit spin to accelerometer range"); self.spin_limit_check.setObjectName("spin_limit_check")
        self.spin_limit_check.setChecked(True)
        self.spin_limit_check.setToolTip(
            "The firmware disarms (ACCELERATION SATURATED) when radial acceleration ω²·r reaches the\n"
            "accelerometer's range (⚙ Physical sensor › Sensor limit). With this on, spin demand is capped\n"
            f"at {SPIN_SENSOR_MARGIN:.0%} of the fastest measurable spin so the motors stay on.\n"
            "Turn it off to test saturation behaviour deliberately.")
        self.spin_limit_check.toggled.connect(self._command_changed)
        form.addRow("", self.spin_limit_check)
        # Backing values for the dial; kept as sliders so recordings and tests
        # address the same command fields.
        self.direction_slider = QSlider(Qt.Orientation.Horizontal); self.direction_slider.setRange(-1800, 1800)
        self.direction_slider.valueChanged.connect(self._command_changed); self.direction_slider.hide()
        self.strength_slider = QSlider(Qt.Orientation.Horizontal); self.strength_slider.setRange(0, 1000)
        self.strength_slider.valueChanged.connect(self._command_changed); self.strength_slider.hide()
        layout.addLayout(form)
        dial_row = QHBoxLayout()
        self.direction_dial = DirectionDial(); self.direction_dial.setObjectName("direction_dial")
        self.direction_dial.changed.connect(self._dial_changed)
        dial_row.addWidget(self.direction_dial, 1)
        legend = QLabel("<b>Translate</b><br><span style='color:#ffd166'>●</span> demand<br>"
                        "<span style='color:#76d5f7'>●</span> actual travel<br><br>"
                        "<span style='color:#7f91a8'>drag to steer<br>shift snaps 15°<br>right-click stops</span>")
        legend.setTextFormat(Qt.TextFormat.RichText); legend.setAlignment(Qt.AlignmentFlag.AlignTop)
        dial_row.addWidget(legend)
        layout.addLayout(dial_row)
        self.demand_label = QLabel("translate 0% @ +0.0° world"); self.demand_label.setObjectName("hint")
        layout.addWidget(self.demand_label)
        self.steering_check = QCheckBox("World-aligned steering"); self.steering_check.setObjectName("steering_check")
        self.steering_check.setChecked(True)
        self.steering_check.setToolTip(
            "On: the dial is in world coordinates and the robot pushes where it points.\n"
            "The controls rotate your demand into the firmware's phase frame, cancelling the\n"
            "phase-estimate drift and the speed-dependent motor/latency lag, then trim the rest\n"
            "from the measured push. This stands in for a driver watching the robot.\n"
            "Off: the dial angle goes to firmware unchanged (raw phase frame).")
        self.steering_check.toggled.connect(self._steering_toggled)
        layout.addWidget(self.steering_check)

        header = QHBoxLayout()
        title = QLabel("TELEMETRY"); title.setObjectName("section_title"); header.addWidget(title)
        header.addStretch()
        self.mode_buttons = QButtonGroup(self)
        for index, (label, tip) in enumerate((
                ("TELEMETRY", "Plot and readouts."),
                ("FLOWCHART", "Block diagram of the control loop with live values on every block and signal.\n"
                              "Hover a block for what it does; click it to open the matching math."))):
            button = QPushButton(label); button.setCheckable(True); button.setToolTip(tip)
            button.setObjectName("mode_button"); button.setChecked(index == 0)
            self.mode_buttons.addButton(button, index); header.addWidget(button)
        layout.addLayout(header)
        self.telemetry_stack = QStackedWidget()
        self.mode_buttons.idToggled.connect(lambda index, on: on and self._set_telemetry_mode(index))
        readouts = QWidget(); readouts_layout = QVBoxLayout(readouts); readouts_layout.setContentsMargins(0, 0, 0, 0)
        self.telemetry = TelemetryPlot(); self.telemetry.setMinimumHeight(200); readouts_layout.addWidget(self.telemetry)
        grid = QFormLayout()
        self.actual_spin = QLabel("0.0 rad/s"); self.estimated_spin = QLabel("0.0 rad/s")
        self.phase = QLabel("0.0 rad"); self.acceleration = QLabel("0.0, 0.0 m/s²")
        self.wheels = QLabel("A 0.000 / B 0.000"); self.forces = QLabel("A 0.00 / B 0.00 N")
        self.faults = QLabel("NONE"); self.faults.setObjectName("faults")
        self.armed_status = QLabel("DISARMED")
        hints = {
            "Actual spin": "Physical truth from the plant.",
            "Estimated spin": "Firmware estimate from the accelerometer: √(radial accel / sensor radius).",
            "Estimated phase": "Firmware's integrated rotation angle; drives translation timing.",
            "Measured acceleration": "Latest accelerometer sample delivered to firmware (sensor frame).",
            "Motor demand": "Firmware throttle outputs, 0..1.",
            "Applied wheel force": "Longitudinal tire force after the traction limit.",
            "Runtime": "Firmware arming state.",
            "Faults": "Active firmware safety faults.",
        }
        for label, widget in (("Actual spin", self.actual_spin), ("Estimated spin", self.estimated_spin),
                              ("Estimated phase", self.phase), ("Measured acceleration", self.acceleration),
                              ("Motor demand", self.wheels), ("Applied wheel force", self.forces),
                              ("Runtime", self.armed_status), ("Faults", self.faults)):
            name = QLabel(label); name.setToolTip(hints[label]); widget.setToolTip(hints[label])
            grid.addRow(name, widget)
        readouts_layout.addLayout(grid); readouts_layout.addStretch()
        self.telemetry_stack.addWidget(readouts)
        self.flow_diagram = FlowDiagram(); self.flow_diagram.setObjectName("flow_diagram")
        self.flow_diagram.block_clicked.connect(self._open_vector)
        self.telemetry_stack.addWidget(self.flow_diagram)
        layout.addWidget(self.telemetry_stack)
        self.record_button = QPushButton("START RECORDING"); self.record_button.setCheckable(True)
        self.record_button.setToolTip("Write this epoch's telemetry to CSV plus a JSON metadata file.")
        self.record_button.toggled.connect(self._toggle_recording); layout.addWidget(self.record_button)
        layout.addStretch()
        return body

    def _settings_dialog(self):
        dialog = QDialog(self); dialog.setObjectName("settings_dialog")
        dialog.setWindowTitle("Settings"); dialog.resize(600, 700)
        layout = QVBoxLayout(dialog)
        header = QHBoxLayout()
        title = QLabel("SETTINGS"); title.setObjectName("section_title"); header.addWidget(title)
        header.addStretch()
        close = QPushButton("✕"); close.setObjectName("settings_close"); close.setFixedWidth(40)
        close.setToolTip("Close settings (Esc). Unapplied edits are kept until you apply or restore.")
        close.clicked.connect(dialog.close); header.addWidget(close)
        self.settings_close_button = close
        layout.addLayout(header)
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
        presets = QHBoxLayout()
        load = QPushButton("LOAD PRESET…"); load.clicked.connect(self._load_preset); presets.addWidget(load)
        save = QPushButton("SAVE PRESET…"); save.clicked.connect(self._save_preset); presets.addWidget(save)
        layout.addLayout(presets)
        note = QLabel("Physical truth and firmware assumptions are independent. Structural changes validate atomically and reset simulation time.")
        note.setWordWrap(True); note.setObjectName("hint"); layout.addWidget(note)
        return dialog

    def _show_settings(self):
        self.settings_dialog.show(); self.settings_dialog.raise_(); self.settings_dialog.activateWindow()

    def _set_telemetry_mode(self, index):
        self.telemetry_stack.setCurrentIndex(index)
        self._refresh_flow()

    def _open_vector(self, key):
        if not self.vectors_check.isChecked():
            self.vectors_check.setChecked(True)
        self.arena.select(key)

    def _refresh_flow(self):
        self.flow_diagram.set_state(self.snapshot, self.config, self.steering.state,
                                    self.spin_slider.value() / 1000.0)

    def _dial_changed(self, angle_deg, strength):
        self.direction_slider.blockSignals(True); self.direction_slider.setValue(round(angle_deg * 10))
        self.direction_slider.blockSignals(False)
        self.strength_slider.setValue(round(strength * 1000))
        self._command_changed()

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

    def _menu(self):
        menu = QMenu(self)
        builtins = sorted(PRESET_DIR.glob("*.json"), key=lambda path: (path.stem.startswith("failure_"), path.stem))
        failures_started = False
        for path in builtins:
            try:
                title = config_codec.metadata(json.loads(path.read_text(encoding="utf-8")))["title"] or path.stem
            except (OSError, json.JSONDecodeError, ValueError):
                continue
            if path.stem.startswith("failure_") and not failures_started:
                failures_started = True
                menu.addSection("Failure modes")
            action = QAction(title, self)
            action.triggered.connect(lambda _=False, preset=path: self._apply_preset_file(preset))
            menu.addAction(action)
        menu.addSeparator()
        load = QAction("Load…", self); load.triggered.connect(self._load_preset); menu.addAction(load)
        save = QAction("Save…", self); save.triggered.connect(self._save_preset); menu.addAction(save)
        self.presets_button.setMenu(menu)

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
        self.steering.reset()
        self._epoch_events.clear(); self._replace_epoch_spool()
        command = self._copy_command(reset_phase=False)
        command.arm = False
        self.command = command
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
        self._update_arm_button()

    def _reset(self):
        self.arm_button.setChecked(False); self.telemetry.clear(); self.reset_requested.emit()

    def _command_changed(self, *_):
        self._send_command(force=True)

    def _send_command(self, force):
        """Build the RC command from the controls. The dial is world-aligned;
        with steering on, its angle is rotated into the firmware's frame."""
        command = native.UserCommand()
        requested_spin = self.spin_slider.value() / 1000.0
        ceiling = self._spin_ceiling()
        limited = self.spin_limit_check.isChecked() and requested_spin > ceiling
        command.spin = ceiling if limited else requested_spin
        strength = self.strength_slider.value() / 1000.0
        desired = math.radians(self.direction_slider.value() / 10.0)
        sent = self.steering.update(self.snapshot, self.config, desired, strength)
        command.translate_x = strength * math.cos(sent)
        command.translate_y = strength * math.sin(sent)
        command.arm = self.arm_button.isChecked()
        previous = self.command
        # Steering re-sends from snapshots only when the sent direction moves
        # noticeably; every send is a recorded command event.
        old_angle = math.atan2(previous.translate_y, previous.translate_x)
        moved = abs(math.remainder(sent - old_angle, 2 * math.pi)) > math.radians(STEERING_RESEND_DEG)
        changed = force or abs(command.spin - previous.spin) > 1e-6 or (strength > 0 and moved)
        self.arena.set_steering(self.steering.state)
        if not changed:
            return
        self.command = command
        self._update_arm_button()
        target = command.spin * self.config.firmware.controller.maximum_spin_rad_s
        if limited:
            self.spin_value.setText(f"<span style='color:#ffd166'>{requested_spin:.0%} → capped {command.spin:.0%} "
                                    f"({target:.0f} rad/s)</span>")
        else:
            self.spin_value.setText(f"{command.spin:.0%} → {target:.0f} rad/s")
        frame = f" (firmware frame {math.degrees(sent):+.1f}°)" if self.steering.enabled and strength > 0 else ""
        self.demand_label.setText(f"translate {strength:.0%} @ {math.degrees(desired):+.1f}° world{frame}")
        self.direction_dial.set_value(math.degrees(desired), strength)
        self.command_requested.emit(command)

    def _steering_toggled(self, enabled):
        self.steering.enabled = enabled
        self.steering.reset()
        self._send_command(force=True)

    def _speed_changed(self, value):
        scale = 10 ** (value / 100.0)
        self.speed_label.setText(f"{scale:.3f}×" if scale < 1 else f"{scale:.2f}×")
        self.time_scale_requested.emit(scale)

    def _update_arm_button(self):
        firmware = self.snapshot.firmware if self.snapshot is not None else None
        if not self.arm_button.isChecked():
            text, tip = "DISARMED", "Press to arm. Spin demand must be at or below the arm limit."
        elif firmware is not None and firmware.armed:
            text, tip = "ARMED", "Firmware is armed and driving the motors."
        elif firmware is not None and int(firmware.faults) & ~int(native.Fault.PHASE_INVALID):
            text, tip = "ARM BLOCKED", "A safety fault is active; see Telemetry → Faults."
        elif self.snapshot is not None and self.snapshot.time_us == 0:
            # No frame consumed yet; the t = 0 frame is disarmed and low, so arming proceeds.
            text, tip = "PRESS START", "The firmware arms once simulation time is running."
        elif self.command.spin > self.config.firmware.arm_spin_max:
            text, tip = "ARM BLOCKED", (
                f"Arming requires spin demand ≤ {self.config.firmware.arm_spin_max:.0%}. "
                "Lower spin, then toggle ARM off and on.")
        elif not self.start_button.isChecked():
            text, tip = "PRESS START", "The firmware arms once simulation time is running."
        elif firmware is not None and not firmware.arm_interlock_satisfied:
            text, tip = "ARM BLOCKED", (
                "The firmware has not seen a disarmed low-spin frame. Lower spin, then toggle ARM off and on.")
        else:
            text, tip = "ARMING…", "Waiting for arm confirmation frames."
        if self.arm_button.text() != text:
            self.arm_button.setText(text)
        self.arm_button.setToolTip(tip)
        if firmware is not None and firmware.armed:
            banner = None
        elif not self.arm_button.isChecked():
            banner = (f"MOTORS OFF · disarmed — set spin ≤ {self.config.firmware.arm_spin_max:.0%}, "
                      "press ARM, then START")
        else:
            banner = "MOTORS OFF · " + (self._last_disarm + ". " if self._last_disarm else "") + tip
        self.arena.set_banner(banner)

    def _track_disarm(self, s, active):
        """Latch why the firmware dropped out of ARMED; faults often clear before
        anyone can read them (e.g. saturation stops once the robot slows)."""
        if s.firmware.armed:
            self._last_disarm = None
        elif self._was_armed and self.arm_button.isChecked():
            # The simulator latches the fault mask at the disarming tick.
            latched = int(s.last_disarm_faults)
            causes = [name for name, fault in FAULTS if latched & int(fault) and name != "PHASE INVALID"]
            reason = " · ".join(causes) if causes else "the arm request"
            self._last_disarm = f"Disarmed by {reason} at t = {s.last_disarm_us / 1e6:.3f} s"
            if "ACCELERATION SATURATED" in causes:
                self._last_disarm += (f" (spin {abs(s.spin_rad_s):.0f} rad/s exceeded the accelerometer's "
                                      f"{self.config.sensor.max_acceleration_mps2:.0f} m/s² range)")
            self.statusBar().showMessage(self._last_disarm, 10000)
        elif not self.arm_button.isChecked():
            self._last_disarm = None
        self._was_armed = s.firmware.armed

    def _spin_ceiling(self):
        """Largest spin fraction whose radial acceleration stays within 90% of
        the accelerometer range: the controller holds ā = ω*²·r_s."""
        controller = self.config.firmware.controller
        measurable = math.sqrt(self.config.sensor.max_acceleration_mps2 / controller.sensor_radius_m)
        return min(1.0, SPIN_SENSOR_MARGIN * measurable / controller.maximum_spin_rad_s)

    def _zero_phase(self):
        self.phase_reset_requested.emit()

    def _copy_command(self, reset_phase=False):
        command = native.UserCommand()
        for name in ("spin", "translate_x", "translate_y", "arm"):
            setattr(command, name, getattr(self.command, name))
        command.reset_phase = reset_phase
        return command

    def _snapshot(self, s):
        self.snapshot = s; self._write_epoch_snapshot(s)
        self.arena.set_snapshot(s); self.telemetry.add_snapshot(s)
        self.direction_dial.set_velocity(s.vx_mps, s.vy_mps)
        self._send_command(force=False)  # keep the steering correction current
        self._refresh_flow()
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
        self._track_disarm(s, active)
        self._update_arm_button()
        if self.record_writer:
            self._write_record_snapshot(s)

    def _command_accepted(self, event):
        self._epoch_events.append(dict(event))

    def _epoch_ending(self, final_snapshot):
        if self.record_writer:
            self._write_record_snapshot(final_snapshot)
            self._finish_recording(final_snapshot)

    def _epoch_reset(self):
        self.steering.reset()
        self._epoch_events.clear(); self._replace_epoch_spool()
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
            self._epoch_spool.flush(); self._epoch_spool.seek(0)
            shutil.copyfileobj(self._epoch_spool, self.record_file)
            self._epoch_spool.seek(0, 2)
            self.record_writer = csv.writer(self.record_file)
            self._recording_document = config_codec.to_document(self.config, self.initial_state)
            self._recording_started_time_us = self.snapshot.time_us if self.snapshot else 0
            self.record_button.setText("STOP RECORDING")
        except OSError as exc:
            self._abort_recording_file()
            self._show_error(f"could not start recording: {exc}")

    def _write_record_snapshot(self, s):
        if not self.record_writer:
            return
        try:
            self.record_writer.writerow(self._snapshot_row(s))
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

    @staticmethod
    def _snapshot_row(s):
        return [s.time_us, s.x_m, s.y_m, s.heading_rad, s.vx_mps, s.vy_mps,
                s.spin_rad_s, s.firmware.controller.phase_rad,
                s.firmware.controller.spin_rad_s, s.firmware.output.wheel_a,
                s.firmware.output.wheel_b, int(s.firmware.faults), int(s.firmware.armed),
                s.wheel_a.applied_force_n, s.wheel_b.applied_force_n]

    def _replace_epoch_spool(self):
        if self._epoch_spool is not None:
            self._epoch_spool.close()
        self._epoch_spool = tempfile.TemporaryFile(mode="w+", newline="", encoding="utf-8")
        self._epoch_spool_writer = csv.writer(self._epoch_spool)
        self._epoch_spool_writer.writerow([
            "time_us", "x_m", "y_m", "heading_rad", "vx_mps", "vy_mps",
            "spin_rad_s", "estimated_phase_rad", "estimated_spin_rad_s",
            "wheel_a_command", "wheel_b_command", "fault_mask", "armed",
            "wheel_a_force_n", "wheel_b_force_n",
        ])

    def _close_epoch_spool(self):
        if self._epoch_spool is not None:
            self._epoch_spool.close()
            self._epoch_spool = self._epoch_spool_writer = None

    def _write_epoch_snapshot(self, snapshot):
        try:
            self._epoch_spool_writer.writerow(self._snapshot_row(snapshot))
        except (OSError, ValueError) as exc:
            self._replace_epoch_spool()
            self._show_error(f"epoch history spool failed: {exc}")

    def _load_preset(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load preset", "", "JSON (*.json)")
        if not path: return
        self._apply_preset_file(path)

    def _apply_preset_file(self, path):
        """Load a preset, apply it, and adopt its UI settings and description."""
        try:
            config, state, meta = config_codec.load_preset(native, path)
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            self._show_error(str(exc)); return
        self._populate_editors(config_codec.to_document(config, state))
        ui = meta["ui"]
        self.steering_check.setChecked(ui.get("world_aligned_steering", True))
        self.spin_limit_check.setChecked(ui.get("limit_spin_to_sensor", True))
        self.apply_requested.emit(config, state)
        title = meta["title"] or Path(path).stem
        self.preset_label.setText(f"<b>{title}</b>" + (f"<br>{meta['description']}" if meta["description"] else ""))
        self.preset_label.show()
        self.statusBar().showMessage(f"Loaded preset: {title}", 5000)

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
        self._close_epoch_spool()
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
QScrollArea#math_overlay { background:rgba(14,21,32,240); border:1px solid #38516d; border-radius:6px; }
QLabel#math_overlay_text { background:transparent; padding:10px; }
QLabel#preset_note { background:#1a2638; border-left:3px solid #ffd166; padding:8px; color:#c9d6e5; }
QPushButton#mode_button { padding:4px 10px; font-size:11px; }
QPushButton#mode_button:checked { background:#29435e; border-color:#76d5f7; color:#76d5f7; }
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
