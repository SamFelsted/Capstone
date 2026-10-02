#pragma once

#include "melty/controller.hpp"
#include "melty/hal.hpp"

namespace melty {

struct RuntimeConfig {
  ControllerConfig controller{};
  Micros control_period_us{1'000};
  Micros acceleration_timeout_us{25'000};
  Micros rc_timeout_us{100'000};
  Micros maximum_tick_interval_us{20'000};
  double arm_spin_max{0.05};
  unsigned arm_confirm_ticks{3};
};

class Runtime {
 public:
  Runtime(Hal& hal, const RuntimeConfig& config = {});

  static bool valid_config(const RuntimeConfig& config);

  // reset() always writes safe output and requires a fresh, valid disarmed RC
  // frame followed by arm_confirm_ticks fresh armed frames before output enables.
  void reset(double phase_reference_rad = 0.0);

  // One non-blocking control transaction: acquire both inputs, validate their
  // timestamps/value domains, update safety state, run control, apply output.
  // Any safety fault or deadline miss writes zero output in this same call.
  void tick();

  const RuntimeStatus& status() const { return status_; }

 private:
  void disarm(Fault fault);

  Hal& hal_;
  RuntimeConfig config_{};
  Controller controller_;
  RuntimeStatus status_{};
  AccelerationSample acceleration_{};
  RcCommand command_{};
  unsigned arm_ticks_{0};
  Micros last_arm_frame_us_{0};
  bool have_arm_frame_{false};
};

}  // namespace melty
