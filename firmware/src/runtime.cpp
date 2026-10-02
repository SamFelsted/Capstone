#include "melty/runtime.hpp"

#include <cmath>

namespace melty {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool valid_acceleration(const AccelerationSample& value) {
  return finite(value.x_mps2) && finite(value.y_mps2);
}

bool valid_command(const RcCommand& value) {
  return finite(value.spin) && value.spin >= 0.0 && value.spin <= 1.0 &&
         finite(value.translate_x) && value.translate_x >= -1.0 &&
         value.translate_x <= 1.0 && finite(value.translate_y) &&
         value.translate_y >= -1.0 && value.translate_y <= 1.0;
}

bool fresh(Micros now, Micros timestamp, Micros timeout) {
  return timestamp <= now && now - timestamp <= timeout;
}

}  // namespace

Runtime::Runtime(Hal& hal, const RuntimeConfig& config)
    : hal_(hal), config_(config), controller_(config.controller) {
  reset();
}

bool Runtime::valid_config(const RuntimeConfig& c) {
  return Controller::valid_config(c.controller) &&
         c.control_period_us > 0 &&
         c.acceleration_timeout_us > 0 && c.rc_timeout_us > 0 &&
         c.maximum_tick_interval_us >= c.control_period_us &&
         finite(c.arm_spin_max) &&
         c.arm_spin_max >= 0.0 && c.arm_spin_max <= 1.0 &&
         c.arm_confirm_ticks > 0;
}

void Runtime::reset(double phase_reference_rad) {
  status_ = {};
  status_.now_us = hal_.now_us();
  status_.last_tick_us = status_.now_us;
  if (!valid_config(config_)) {
    status_.faults = Fault::invalid_configuration;
  }
  acceleration_ = {};
  command_ = {};
  arm_ticks_ = 0;
  last_arm_frame_us_ = 0;
  have_arm_frame_ = false;
  controller_.reset(status_.now_us, phase_reference_rad);
  status_.controller = controller_.telemetry();
  hal_.write_motors({});
}

void Runtime::disarm(Fault fault) {
  status_.faults |= fault;
  status_.armed = false;
  status_.arm_interlock_satisfied = false;
  arm_ticks_ = 0;
  last_arm_frame_us_ = command_.timestamp_us;
  have_arm_frame_ = true;
  controller_.disable_output();
  status_.output = {};
}

void Runtime::tick() {
  const Micros now = hal_.now_us();
  Fault active_faults = Fault::none;
  if (!valid_config(config_)) {
    active_faults |= Fault::invalid_configuration;
  }
  if (now < status_.last_tick_us ||
      now - status_.last_tick_us > config_.maximum_tick_interval_us) {
    active_faults |= Fault::control_deadline;
  }

  AccelerationSample new_acceleration{};
  if (!hal_.read_acceleration(new_acceleration)) {
    active_faults |= Fault::hal_error;
  } else if (!valid_acceleration(new_acceleration)) {
    active_faults |= Fault::invalid_acceleration;
  } else {
    acceleration_ = new_acceleration;
    status_.last_acceleration_us = acceleration_.timestamp_us;
    if (acceleration_.saturated) {
      active_faults |= Fault::acceleration_saturated;
    }
  }
  if (!fresh(now, acceleration_.timestamp_us,
             config_.acceleration_timeout_us)) {
    active_faults |= Fault::stale_acceleration;
  }

  RcCommand new_command{};
  if (!hal_.read_rc(new_command)) {
    active_faults |= Fault::hal_error;
  } else if (!valid_command(new_command)) {
    active_faults |= Fault::invalid_rc;
  } else {
    command_ = new_command;
    status_.last_rc_us = command_.timestamp_us;
  }
  if (!fresh(now, command_.timestamp_us, config_.rc_timeout_us)) {
    active_faults |= Fault::stale_rc;
  }

  status_.now_us = now;
  status_.faults = active_faults;
  const Fault fatal_faults =
      Fault::invalid_configuration | Fault::invalid_acceleration |
      Fault::stale_acceleration | Fault::invalid_rc | Fault::stale_rc |
      Fault::control_deadline | Fault::hal_error;
  const Fault all_fatal_faults = fatal_faults | Fault::acceleration_saturated;
  if (any(active_faults & all_fatal_faults)) {
    disarm(active_faults);
  } else {
    if (!command_.hardware_enabled) {
      status_.armed = false;
      status_.arm_interlock_satisfied = false;
      arm_ticks_ = 0;
      last_arm_frame_us_ = command_.timestamp_us;
      have_arm_frame_ = true;
    } else if (!command_.arm) {
      status_.armed = false;
      arm_ticks_ = 0;
      if (!have_arm_frame_ || command_.timestamp_us != last_arm_frame_us_) {
        status_.arm_interlock_satisfied =
            command_.spin <= config_.arm_spin_max;
        last_arm_frame_us_ = command_.timestamp_us;
        have_arm_frame_ = true;
      }
      if (command_.reset_phase) {
        controller_.reset(now, 0.0);
      }
    } else if (!status_.arm_interlock_satisfied) {
      status_.armed = false;
      arm_ticks_ = 0;
    } else if (!status_.armed) {
      if (!have_arm_frame_ || command_.timestamp_us != last_arm_frame_us_) {
        ++arm_ticks_;
        last_arm_frame_us_ = command_.timestamp_us;
        have_arm_frame_ = true;
      }
      if (arm_ticks_ >= config_.arm_confirm_ticks) {
        status_.armed = true;
      }
    }

    MotorOutput requested =
        controller_.update(now, acceleration_, command_, status_.armed);
    if (!controller_.numeric_valid()) {
      disarm(Fault::controller_numeric);
      requested = {};
    } else if (!controller_.telemetry().phase_valid) {
      status_.faults |= Fault::phase_invalid;
    }
    status_.output = requested;
  }

  status_.controller = controller_.telemetry();
  hal_.write_motors(status_.output);
  status_.last_tick_us = now;
}

}  // namespace melty
