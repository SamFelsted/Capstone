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
