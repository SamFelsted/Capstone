#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "sim/simulator.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool has(melty::Fault all, melty::Fault flag) {
  return melty::any(all & flag);
}

void establish_interlock(melty::sim::Simulator& simulator) {
  simulator.set_command({0.0, 0.0, 0.0, false, false});
  simulator.advance_for(10'000);
}

}  // namespace

int main() {
  melty::sim::SimulationConfig config{};
  config.sensor.noise_stddev_mps2 = 0.0;
  config.firmware.controller.translation_phase_offset_rad = 2.15;
  melty::sim::Simulator simulator(config);
  establish_interlock(simulator);
  simulator.set_command({0.25, 0.0, 0.0, true, false});
  simulator.advance_for(2'000'000);
  auto spun = simulator.snapshot();
  require(spun.firmware.armed,
          "closed loop should arm after disarmed and distinct armed frames");
  require(spun.spin_rad_s > 20.0,
          "phase-invalid startup must physically spin the chassis from rest");
  require(spun.firmware.controller.phase_valid,
          "physical radial acceleration should establish estimator phase");
  require(std::abs(std::remainder(spun.firmware.controller.phase_rad -
                                      spun.heading_rad,
                                  2.0 * 3.14159265358979323846)) > 0.1,
          "realistic sensing and filtering should expose phase error honestly");
  require(spun.wheel_a.motor_current_a >= 0.0 &&
              spun.wheel_b.motor_current_a >= 0.0,
          "forward-only ESC model must not command regenerative current");

  const double start_x = spun.x_m;
  const double start_y = spun.y_m;
  simulator.set_command({0.25, 0.8, 0.0, true, false});
  simulator.advance_for(2'000'000);
  const auto translated = simulator.snapshot();
  const double travel_x = translated.x_m - start_x;
  const double travel_y = translated.y_m - start_y;
  require(travel_x > 0.02 && std::abs(travel_y) < 0.5 * travel_x,
          "calibrated +X modulation should produce predominantly +X travel");

  // The same directional contract holds with idealized zero transport delay;
  // no physical heading is fed back into firmware during the run.
  melty::sim::SimulationConfig ideal_config = config;
  ideal_config.sensor.latency_us = 0;
  ideal_config.command_latency_us = 0;
  ideal_config.actuator_latency_us = 0;
  melty::sim::Simulator ideal(ideal_config);
  establish_interlock(ideal);
  ideal.set_command({0.25, 0.0, 0.0, true, false});
  ideal.advance_for(2'000'000);
  const auto ideal_start = ideal.snapshot();
  ideal.set_command({0.25, 0.8, 0.0, true, false});
  ideal.advance_for(2'000'000);
  const auto ideal_end = ideal.snapshot();
  const double ideal_x = ideal_end.x_m - ideal_start.x_m;
  const double ideal_y = ideal_end.y_m - ideal_start.y_m;
  require(ideal_x > 0.02 && std::abs(ideal_y) < 0.5 * ideal_x,
          "ideal zero-delay +X demand should remain aligned with +X travel");

  // Reset must reproduce the exact seeded measurement stream.
  simulator.reset();
  simulator.advance_for(20'000);
  const auto first = simulator.snapshot().sensed_acceleration;
  simulator.reset();
  simulator.advance_for(20'000);
  const auto second = simulator.snapshot().sensed_acceleration;
  require(first.x_mps2 == second.x_mps2 && first.y_mps2 == second.y_mps2,
          "reset should reproduce deterministic seeded sensor noise");

  melty::sim::SimulationConfig delayed_config{};
  delayed_config.sensor.noise_stddev_mps2 = 0.0;
  delayed_config.command_latency_us = 20'000;
  melty::sim::Simulator delayed(delayed_config);
  establish_interlock(delayed);
  delayed.set_command({0.25, 0.0, 0.0, true, false});
  delayed.advance_for(10'000);
  require(!delayed.snapshot().firmware.armed,
          "radio command must not arrive before configured latency");
  delayed.advance_for(30'000);
  require(delayed.snapshot().firmware.armed,
          "fresh periodic radio frames should arm after transport latency");

  bool rejected_nan = false;
  try {
    melty::sim::UserCommand invalid{};
    invalid.spin = std::numeric_limits<double>::quiet_NaN();
    simulator.set_command(invalid);
  } catch (const std::invalid_argument&) {
    rejected_nan = true;
  }
  require(rejected_nan, "non-finite user commands must be rejected");

  melty::sim::SimulationConfig saturated_config{};
  saturated_config.sensor.noise_stddev_mps2 = 0.0;
  saturated_config.sensor.max_acceleration_mps2 = 15.0;
  melty::sim::Simulator saturated(saturated_config);
  establish_interlock(saturated);
  saturated.set_command({0.25, 0.0, 0.0, true, false});
  bool observed_saturation = false;
  for (int i = 0; i < 1000; ++i) {
    saturated.advance_for(1'000);
    observed_saturation =
        observed_saturation ||
        has(saturated.snapshot().firmware.faults,
            melty::Fault::acceleration_saturated);
  }
  require(observed_saturation,
          "clipped simulated accelerometer must report saturation fault");
  require(!saturated.snapshot().firmware.armed,
          "accelerometer saturation must disarm the runtime");

  std::cout << "closed_loop_test passed\n";
  return 0;
}
