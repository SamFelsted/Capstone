#pragma once

#include "melty/types.hpp"

namespace melty {

// HAL owns timestamp acquisition. now_us() and sample timestamps must share one
// monotonic epoch. Board adapters must extend wrapping hardware counters.
class Hal {
 public:
  virtual ~Hal() = default;

  virtual Micros now_us() = 0;
  virtual bool read_acceleration(AccelerationSample& sample) = 0;
  virtual bool read_rc(RcCommand& command) = 0;
  virtual void write_motors(const MotorOutput& output) = 0;
};

}  // namespace melty
