#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "sim/plant.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

double kinetic_energy(const melty::sim::PhysicalConfig& config,
                      const melty::sim::Snapshot& state) {
  const double chassis =
      0.5 * config.mass_kg *
          (state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps) +
      0.5 * config.moment_of_inertia_kg_m2 *
          state.spin_rad_s * state.spin_rad_s;
  const double motor =
      0.5 * config.motor_inertia_kg_m2 * config.gear_ratio *
      config.gear_ratio *
      (state.wheel_a.wheel_speed_rad_s *
           state.wheel_a.wheel_speed_rad_s +
       state.wheel_b.wheel_speed_rad_s *
           state.wheel_b.wheel_speed_rad_s);
  return chassis + motor;
}

}  // namespace

int main() {
  melty::sim::PhysicalConfig config{};
  config.linear_drag_n_per_mps = 0.0;

  melty::sim::Plant symmetric(config);
  symmetric.reset({});
  for (int i = 0; i < 8000; ++i) {
    symmetric.step(0.35, 0.35, 0.00025);
  }
  const auto spun = symmetric.snapshot(2'000'000);
  require(spun.spin_rad_s > 20.0,
          "equal forward wheel commands should spin from rest");
  require(std::hypot(spun.x_m, spun.y_m) < 1.0e-6,
          "opposed symmetric wheel forces should not translate the center");
  require(std::abs(spun.wheel_a.wheel_speed_rad_s -
                   spun.wheel_b.wheel_speed_rad_s) < 1.0e-9,
          "symmetric drivetrains should retain equal wheel speed");

  melty::sim::Plant coasting(config);
  melty::sim::ResetState moving{};
  moving.vx_mps = 1.2;
  moving.vy_mps = -0.4;
  moving.spin_rad_s = 80.0;
  coasting.reset(moving);
  double previous_energy = kinetic_energy(config, coasting.snapshot(0));
  for (int block = 0; block < 200; ++block) {
    for (int i = 0; i < 40; ++i) {
      coasting.step(0.0, 0.0, 0.00025);
    }
    const double energy = kinetic_energy(config, coasting.snapshot(0));
    require(energy <= previous_energy + 1.0e-7,
            "unpowered tire, drag, and motor dynamics must not create energy");
    previous_energy = energy;
  }

  melty::sim::Plant fine(config);
  melty::sim::Plant coarse(config);
  fine.reset({});
  coarse.reset({});
  for (int i = 0; i < 4000; ++i) fine.step(0.28, 0.28, 0.000125);
  for (int i = 0; i < 1000; ++i) coarse.step(0.28, 0.28, 0.0005);
  const auto fine_state = fine.snapshot(500'000);
  const auto coarse_state = coarse.snapshot(500'000);
  require(std::abs(fine_state.spin_rad_s - coarse_state.spin_rad_s) < 1.0,
          "spin solution should converge across practical outer timesteps");

  const auto before_rejected_reset = coarse.snapshot(0);
  melty::sim::ResetState unsafe{};
  unsafe.spin_rad_s = std::numeric_limits<double>::max();
  bool direct_reset_rejected = false;
  try {
    coarse.reset(unsafe);
  } catch (const std::invalid_argument&) {
    direct_reset_rejected = true;
  }
  const auto after_rejected_reset = coarse.snapshot(0);
  require(direct_reset_rejected &&
              before_rejected_reset.spin_rad_s ==
                  after_rejected_reset.spin_rad_s &&
              before_rejected_reset.x_m == after_rejected_reset.x_m,
          "Plant reset must reject unsafe magnitudes without mutating state");

  std::size_t substeps = 0;
  melty::sim::PhysicalConfig unstable = config;
  unstable.moment_of_inertia_kg_m2 = 1.0e-9;
  unstable.tire_longitudinal_stiffness_n_per_mps = 1.0e9;
  require(!melty::sim::Plant::substep_count(unstable, 0.00025, substeps),
          "yaw-stiff extreme must exceed the bounded substep budget");

  std::cout << "plant_test passed\n";
  return 0;
}
