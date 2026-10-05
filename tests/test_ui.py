import json
import os
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

melty_sim = pytest.importorskip("melty_sim")
pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication

from sim.ui import config_codec
from sim.ui.main import MainWindow


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


def spin_events(app, duration_ms=120):
    deadline = time.monotonic() + duration_ms / 1000
    while time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.002)


def test_reference_presets_match_native_codec():
    for name in ("robot.json", "scenario.json"):
        document = json.loads((Path(__file__).parents[1] / "sim" / "config" / name).read_text())
        config, state = config_codec.from_document(melty_sim, document)
        assert not melty_sim.Simulator.validate(config)
        assert config_codec.to_document(config, state)["version"] == 1


def test_invalid_preset_is_atomic_and_finite():
    config, state = melty_sim.SimulationConfig(), melty_sim.ResetState()
    document = config_codec.to_document(config, state)
    document["config"]["physical"]["mass_kg"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        config_codec.from_document(melty_sim, document)
    assert config.physical.mass_kg == pytest.approx(1.36)


def test_integer_widths_and_reset_state_are_rejected_as_value_errors():
    config, state = melty_sim.SimulationConfig(), melty_sim.ResetState()
    document = config_codec.to_document(config, state)
    document["config"]["physics_tick_us"] = -1
    with pytest.raises(ValueError, match="physics_tick_us"):
        config_codec.from_document(melty_sim, document)
    document = config_codec.to_document(config, state)
    document["config"]["scenario_seed"] = 10 ** 100
    with pytest.raises(ValueError, match="scenario_seed"):
        config_codec.from_document(melty_sim, document)
    document = config_codec.to_document(config, state)
    document["initial_state"]["x_m"] = 1.0e7
    with pytest.raises(ValueError, match="initial_state"):
        config_codec.from_document(melty_sim, document)
    document = config_codec.to_document(config, state)
    document["config"]["physical"]["mass_kg"] = 10 ** 1000
    with pytest.raises(ValueError, match="mass_kg"):
        config_codec.from_document(melty_sim, document)


def test_simulator_config_property_is_a_deep_copy():
    simulator = melty_sim.Simulator()
    exposed = simulator.config
    exposed.physics_tick_us = 999
    exposed.physical.mass_kg = 99.0
    assert simulator.config.physics_tick_us == 250
    assert simulator.config.physical.mass_kg == pytest.approx(1.36)
    simulator.advance_ticks(1)
    assert simulator.snapshot().time_us == 250


def test_controls_apply_reset_and_worker_shutdown(app):
    window = MainWindow()
    window.show()
    spin_events(app, 180)
    assert window.snapshot is not None
    mass = window.findChild(type(window.editors["physical.mass_kg"]), "field_physical_mass_kg")
    mass.setText("1.5")
    window.apply_button.click()
    spin_events(app, 180)
    assert window.config.physical.mass_kg == pytest.approx(1.5)
    assert window.snapshot.time_us == 0
    assert not hasattr(window, "_epoch_snapshots")
    window._epoch_spool.flush()
    assert window._epoch_spool.tell() > 0
    started = time.monotonic()
    assert window.close()
    assert time.monotonic() - started < 0.5
    assert not window.thread.isRunning()
    spin_events(app, 30)
    assert window.worker_timer_destroyed
    assert window._epoch_spool is None


def test_editor_integer_errors_and_invalid_load_preserve_state(app, tmp_path, monkeypatch):
    window = MainWindow(); spin_events(app, 100)
    original_config = window.config.physics_tick_us
    original_text = window.editors["physical.mass_kg"].text()
    window.editors["config.physics_tick_us"].setText("-1")
    window.apply_button.click()
    assert window.config.physics_tick_us == original_config
    assert "physics_tick_us" in window.statusBar().currentMessage()
    window.editors["config.physics_tick_us"].setText(str(original_config))
    window.editors["config.scenario_seed"].setText(str(10 ** 100))
    window.apply_button.click()
    assert window.config.scenario_seed == 1
    assert "scenario_seed" in window.statusBar().currentMessage()

    bad = config_codec.to_document(melty_sim.SimulationConfig(), melty_sim.ResetState())
    bad["initial_state"]["x_m"] = 1.0e7
    path = tmp_path / "bad.json"; path.write_text(json.dumps(bad))
    monkeypatch.setattr("sim.ui.main.QFileDialog.getOpenFileName",
                        lambda *args: (str(path), "JSON (*.json)"))
    window._load_preset()
    assert window.editors["physical.mass_kg"].text() == original_text
    assert window.config.physics_tick_us == original_config
    assert window.close()


def test_apply_while_arm_requested_cannot_resurrect_arm_on_phase_reset(app):
    window = MainWindow(); spin_events(app, 100)
    window.arm_button.setChecked(True); spin_events(app, 50)
    window.apply_document(config_codec.to_document(window.config, window.initial_state))
    spin_events(app, 120)
    assert not window.arm_button.isChecked()
    assert not window.command.arm
    window._epoch_events.clear()
    before = window.snapshot.time_us
    window._zero_phase(); spin_events(app, 100)
    phase_events = window._epoch_events[-2:]
    assert [event["reset_phase"] for event in phase_events] == [True, False]
    assert all(not event["arm"] for event in phase_events)
    assert window.snapshot.time_us > before
    assert window.close()


@pytest.mark.parametrize("latency_us", [2_000, 1])
def test_paused_phase_reset_hits_absolute_control_boundary_and_replays(app, latency_us):
    window = MainWindow(); spin_events(app, 100)
    document = config_codec.to_document(window.config, window.initial_state)
    document["config"]["firmware"]["control_period_us"] = 15_000
    document["config"]["command_latency_us"] = latency_us
    document["initial_state"]["estimated_phase_rad"] = 1.0
    window.apply_document(document); spin_events(app, 120)
    assert window.snapshot.firmware.controller.phase_rad == pytest.approx(1.0)
    window._epoch_events.clear(); before = window.snapshot.time_us
    window._zero_phase(); spin_events(app, 120)
    events = list(window._epoch_events)
    assert [e["reset_phase"] for e in events] == [True, False]
    assert window.snapshot.time_us == 30_000
    assert window.snapshot.firmware.controller.phase_rad == pytest.approx(0.0, abs=1e-6)
    assert "Rejected:" not in window.statusBar().currentMessage()

    config, state = config_codec.from_document(melty_sim, document)
    replay = melty_sim.Simulator(config); replay.reset(state)
    cursor = 0
    for event in events:
        replay.advance_for(event["time_us"] - cursor); cursor = event["time_us"]
        command = melty_sim.UserCommand()
        for name in ("spin", "translate_x", "translate_y", "arm", "reset_phase"):
            setattr(command, name, event[name])
        replay.set_command(command)
    replay.advance_for(window.snapshot.time_us - cursor)
    replayed = replay.snapshot()
    assert replayed.firmware.controller.phase_rad == pytest.approx(
        window.snapshot.firmware.controller.phase_rad, abs=1e-10)
    for name in ("x_m", "y_m", "heading_rad", "vx_mps", "vy_mps", "spin_rad_s"):
        assert getattr(replayed, name) == pytest.approx(
            getattr(window.snapshot, name), abs=1e-10)
    assert window.close()


def test_phase_reset_reports_sensor_fault_and_still_clears_pulse(app):
    window = MainWindow(); spin_events(app, 100)
    document = config_codec.to_document(window.config, window.initial_state)
    document["config"]["sensor"]["max_acceleration_mps2"] = 1.0e-6
    document["initial_state"]["estimated_phase_rad"] = 1.0
    window.apply_document(document); spin_events(app, 100)
    window._epoch_events.clear()
    window._zero_phase(); spin_events(app, 100)
    assert [event["reset_phase"] for event in window._epoch_events] == [True, False]
    assert "blocking fault" in window.statusBar().currentMessage()
    assert not window.start_button.isChecked()
    assert window.close()


def test_phase_reset_rejects_long_synchronous_window_before_asserting(app):
    window = MainWindow(); spin_events(app, 100)
    document = config_codec.to_document(window.config, window.initial_state)
    document["config"]["physics_tick_us"] = 1
    document["config"]["firmware"]["control_period_us"] = 101
    document["config"]["command_latency_us"] = 10_000_000
    window.apply_document(document); spin_events(app, 120)
    assert window.snapshot.time_us == 0
    window._epoch_events.clear()
    original = (window.command.spin, window.command.translate_x,
                window.command.translate_y, window.command.arm,
                window.command.reset_phase)
    window._zero_phase(); spin_events(app, 100)
    assert window.snapshot.time_us == 0
    assert window._epoch_events == []
    assert (window.command.spin, window.command.translate_x,
            window.command.translate_y, window.command.arm,
            window.command.reset_phase) == original
    message = window.statusBar().currentMessage()
    assert "exceeds the 10000-tick action budget" in message
    assert "Apply & Reset" in message
    assert window.close()


def test_rapid_commands_are_copied_and_acknowledged_in_order(app):
    window = MainWindow(); spin_events(app, 120)
    window.spin_limit_check.setChecked(False); spin_events(app, 60)  # transport test: raw values
    window._epoch_events.clear()
    for value in (110, 420, 870):
        window.spin_slider.setValue(value)
    spin_events(app, 120)
    assert [round(e["spin"], 2) for e in window._epoch_events] == [0.11, 0.42, 0.87]
    assert all(set(e) == {"time_us", "spin", "translate_x", "translate_y", "arm", "reset_phase"}
               for e in window._epoch_events)
    assert window.close()


def test_recording_replays_to_final_state(app, tmp_path):
    window = MainWindow(); spin_events(app, 120)
    window.arm_button.setChecked(True)
    window.start_button.click()
    spin_events(app, 100)
    window.spin_slider.setValue(250)
    spin_events(app, 180)
    path = tmp_path / "replay.csv"
    window._start_recording(path)  # starts mid-epoch and writes prehistory
    window.strength_slider.setValue(350)
    window.direction_slider.setValue(300)
    spin_events(app, 180)
    window.start_button.click()
    spin_events(app, 60)
    window.step_button.click(); spin_events(app, 60)
    window.recording_boundary_requested.emit(); spin_events(app, 60)

    metadata = json.loads(path.with_suffix(".json").read_text())
    rows = path.read_text().splitlines()
    assert len(rows) > 3
    assert int(rows[1].split(",")[0]) == 0  # history starts at epoch reset
    config, state = config_codec.from_recording(melty_sim, metadata)
    simulator = melty_sim.Simulator(config); simulator.reset(state)
    cursor = 0
    for event in metadata["command_events"]:
        assert event["time_us"] >= cursor
        simulator.advance_for(event["time_us"] - cursor)
        cursor = event["time_us"]
        command = melty_sim.UserCommand()
        for name in ("spin", "translate_x", "translate_y", "arm", "reset_phase"):
            setattr(command, name, event[name])
        simulator.set_command(command)
    simulator.advance_for(metadata["final_time_us"] - cursor)
    replayed = simulator.snapshot()
    for name, expected in metadata["final_state"].items():
        assert getattr(replayed, name) == pytest.approx(expected, abs=1e-10)
    assert window.close()


def test_arm_before_start_spins_up_and_speed_slider_slows_time(app):
    window = MainWindow(); spin_events(app, 100)
    window.arm_button.setChecked(True); spin_events(app, 50)
    assert window.arm_button.text() == "PRESS START"
    window.start_button.click(); spin_events(app, 300)
    assert window.snapshot.firmware.armed
    assert window.arm_button.text() == "ARMED"
    window.spin_slider.setValue(250); spin_events(app, 1000)
    assert window.snapshot.spin_rad_s > 20.0

    window.speed_slider.setValue(-200); spin_events(app, 100)
    assert window.speed_label.text() == "0.010×"
    before = window.snapshot.time_us
    spin_events(app, 500)
    # 0.5 s of wall time at 0.01x is about 5 ms of simulated time.
    assert 0 < window.snapshot.time_us - before < 50_000

    window.vectors_check.setChecked(False)
    assert not window.arena.show_vectors
    assert window.close()


def test_vector_derivations_match_native_values():
    from sim.ui import vector_math
    config, state = config_codec.load(melty_sim, Path(__file__).parents[1] / "sim" / "config" / "scenario.json")
    simulator = melty_sim.Simulator(config); simulator.reset(state)
    command = melty_sim.UserCommand(); command.arm = True
    simulator.set_command(command); simulator.advance_for(20_000)
    command.spin = 0.25; command.translate_x = 0.6; command.translate_y = 0.3
    simulator.set_command(command)
    checked = 0
    for _ in range(400):
        simulator.advance_ticks(11)
        snapshot = simulator.snapshot()
        for key in vector_math.VECTORS:
            derivation = vector_math.derive(key, snapshot, config)
            assert vector_math.to_html(derivation)
            assert all(section.why for section in derivation.sections), (key, [s.title for s in derivation.sections])
            for check in derivation.checks:
                assert check.ok, (key, check.name, check.recomputed, check.reported)
                checked += 1
    assert snapshot.firmware.armed and checked > 0


def test_direction_dial_drives_translation_command(app):
    window = MainWindow(); spin_events(app, 100)
    window._dial_changed(90.0, 0.5)
    assert window.command.translate_x == pytest.approx(0.0, abs=1e-9)
    assert window.command.translate_y == pytest.approx(0.5)
    window.direction_slider.setValue(-1800); window.strength_slider.setValue(1000)
    assert window.direction_dial.angle_deg == pytest.approx(-180.0)
    assert window.direction_dial.strength == pytest.approx(1.0)
    assert window.close()


def test_clicking_vectors_opens_live_math_and_settings_live_in_dialog(app):
    window = MainWindow(); window.show(); spin_events(app, 150)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 150)
    window.spin_slider.setValue(250); spin_events(app, 400)
    window.arena.repaint()
    midpoints = {key: (a + b) / 2 for key, a, b in reversed(window.arena._hits)}
    assert set(midpoints) == {"wheel_a", "wheel_b", "net_force", "velocity", "steering"}
    assert window.arena._hit(midpoints["wheel_a"]) == "wheel_a"
    window.arena.select("wheel_a"); spin_events(app, 100)
    assert window.arena.overlay.isVisible()
    assert "Wheel A motor command" in window.arena.overlay_text.text()
    window.vectors_check.setChecked(False)
    assert not window.arena.overlay.isVisible()
    assert window.editors["physical.mass_kg"].window() is window.settings_dialog
    assert window.close()


def test_realtime_button_and_motors_off_banner(app):
    window = MainWindow(); window.show(); spin_events(app, 100)
    window.speed_slider.setValue(-150)
    window.realtime_button.click()
    assert window.speed_slider.value() == 0 and window.speed_label.text() == "1.00×"
    window.start_button.click(); spin_events(app, 100)
    window.spin_slider.setValue(500); spin_events(app, 100)  # firmware sees a disarmed high-spin frame
    window.arm_button.setChecked(True); spin_events(app, 200)
    assert not window.snapshot.firmware.armed
    assert window.arm_button.text() == "ARM BLOCKED"
    assert window.arena.banner and "spin demand" in window.arena.banner
    window.arena.repaint()
    assert {key for key, _ in window.arena._legend_hits} == {"wheel_a", "wheel_b", "net_force", "velocity", "steering"}
    window._show_settings(); spin_events(app, 50)
    window.settings_close_button.click(); spin_events(app, 50)
    assert not window.settings_dialog.isVisible()
    assert window.close()


def test_world_aligned_steering_pushes_where_the_dial_points(app):
    import math
    window = MainWindow(); window.show(); spin_events(app, 100)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 100)
    window.spin_slider.setValue(250); spin_events(app, 1500)
    window._dial_changed(30.0, 1.0)
    start = (window.snapshot.x_m, window.snapshot.y_m)
    spin_events(app, 2500)
    state = window.steering.state
    assert state.push_rad is not None
    assert abs(math.degrees(math.remainder(state.push_rad - math.radians(30.0), 2 * math.pi))) < 10.0
    moved = (window.snapshot.x_m - start[0], window.snapshot.y_m - start[1])
    assert math.hypot(*moved) > 0.005
    assert abs(math.degrees(math.remainder(math.atan2(moved[1], moved[0]) - math.radians(30.0), 2 * math.pi))) < 35.0
    window.arena.select("steering"); spin_events(app, 50)
    assert "World → firmware frame" in window.arena.overlay_text.text()
    assert window.close()


def test_spin_limiter_keeps_motors_on_and_disarm_reason_is_latched(app):
    window = MainWindow(); window.show(); spin_events(app, 100)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 100)
    window.spin_slider.setValue(790); spin_events(app, 4000)
    ceiling = window._spin_ceiling()
    assert window.command.spin == pytest.approx(ceiling)
    assert window.snapshot.firmware.armed
    assert "capped" in window.spin_value.text()

    window.spin_limit_check.setChecked(False); spin_events(app, 3000)
    assert not window.snapshot.firmware.armed
    assert "ACCELERATION SATURATED" in (window._last_disarm or "")
    assert window.arena.banner and "ACCELERATION SATURATED" in window.arena.banner
    assert window.close()


def test_builtin_presets_load_from_menu_with_ui_settings(app):
    from sim.ui.main import PRESET_DIR
    window = MainWindow(); spin_events(app, 100)
    titles = [action.text() for action in window.presets_button.menu().actions() if action.text()]
    assert "Failure: realistic accelerometer" in titles and "Default robot" in titles
    for path in sorted(PRESET_DIR.glob("*.json")):
        meta = config_codec.metadata(json.loads(path.read_text()))
        window._apply_preset_file(path); spin_events(app, 80)
        assert not window.preset_label.isHidden() and meta["title"] in window.preset_label.text()
        assert window.steering_check.isChecked() == meta["ui"].get("world_aligned_steering", True)
        assert window.spin_limit_check.isChecked() == meta["ui"].get("limit_spin_to_sensor", True)
    assert window.close()


def test_low_range_preset_disarms_with_latched_saturation(app):
    from sim.ui.main import PRESET_DIR
    window = MainWindow(); window.show(); spin_events(app, 100)
    window._apply_preset_file(PRESET_DIR / "failure_low_range_accelerometer.json"); spin_events(app, 150)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 150)
    window.spin_slider.setValue(300); spin_events(app, 2500)
    assert not window.snapshot.firmware.armed
    assert "ACCELERATION SATURATED" in (window._last_disarm or "")
    assert window.close()


def test_steady_steering_does_not_flood_command_events(app):
    window = MainWindow(); window.show(); spin_events(app, 100)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 100)
    window.spin_slider.setValue(250); window._dial_changed(30.0, 0.8); spin_events(app, 2500)
    before = len(window._epoch_events)
    spin_events(app, 3000)
    assert len(window._epoch_events) - before < 30  # ~1/s steady; was ~21/s
    assert window.close()


def test_flowchart_mode_shows_live_blocks_and_opens_math(app):
    from PySide6.QtCore import QPointF
    window = MainWindow(); window.show(); spin_events(app, 100)
    window.arm_button.setChecked(True); window.start_button.click(); spin_events(app, 100)
    window.spin_slider.setValue(250); spin_events(app, 600)
    window.mode_buttons.button(1).click(); spin_events(app, 100)
    assert window.telemetry_stack.currentWidget() is window.flow_diagram
    window.flow_diagram.repaint()
    diagram = window.flow_diagram
    assert {"mixer", "tires", "chassis", "accelerometer"} <= set(diagram._rects)
    mixer_lines = diagram._lines("mixer")
    assert mixer_lines[0] == f"u_A = {window.snapshot.firmware.output.wheel_a:.3f}"
    received = []
    diagram.block_clicked.connect(received.append)
    tires = diagram._rects["tires"].center()
    assert diagram._block_at(QPointF(tires)) == "tires"
    window._open_vector("net_force"); spin_events(app, 50)
    assert window.arena.selected == "net_force"
    window.mode_buttons.button(0).click()
    assert window.telemetry_stack.currentIndex() == 0
    assert window.presets_button.menu() is not None
    assert window.close()
