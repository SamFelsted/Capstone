#pragma once

#include "sim/motor.hpp"
#include "sim/simulator.hpp"

namespace melty::sim {

struct PlantAcceleration {
  double body_x_mps2{0.0};
  double body_y_mps2{0.0};
  double angular_rad_s2{0.0};
};

class Plant {
 public:
  explicit Plant(const PhysicalConfig& config);
  ~Plant();
  Plant(Plant&&) noexcept;
  Plant& operator=(Plant&&) noexcept;
  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;

  void reset(const ResetState& state);
  void step(double wheel_a_command, double wheel_b_command, double dt_s);

  Snapshot snapshot(Micros now_us) const;
  PlantAcceleration acceleration() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace melty::sim
