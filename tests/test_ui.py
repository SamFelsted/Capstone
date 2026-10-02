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
    started = time.monotonic()
    assert window.close()
    assert time.monotonic() - started < 0.5
    assert not window.thread.isRunning()
    spin_events(app, 30)
    assert window.worker_timer_destroyed


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
