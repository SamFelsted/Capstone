#pragma once

#include "melty/hal.hpp"

namespace melty::sim {

// Host HAL used by Simulator. Samples are published only after their simulated
// transport latency has elapsed; Runtime therefore sees exactly what firmware
// would see, rather than physical truth.
class SimHal final : public Hal {
 public:
  Micros now_us() override;
  bool read_acceleration(AccelerationSample& sample) override;
  bool read_rc(RcCommand& command) override;
  void write_motors(const MotorOutput& output) override;

  void reset();
  void set_now(Micros now_us);
  void publish_acceleration(const AccelerationSample& sample);
  void publish_rc(const RcCommand& command);
  const MotorOutput& motor_output() const;
  const AccelerationSample* delivered_acceleration() const;

 private:
  Micros now_us_{0};
  AccelerationSample acceleration_{};
  RcCommand command_{};
  MotorOutput motors_{};
  bool acceleration_available_{false};
  bool command_available_{false};
};

}  // namespace melty::sim
