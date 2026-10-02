#include "sim/sim_hal.hpp"

namespace melty::sim {

Micros SimHal::now_us() { return now_us_; }

bool SimHal::read_acceleration(AccelerationSample& sample) {
  if (!acceleration_available_) {
    return false;
  }
  sample = acceleration_;
  return true;
}

bool SimHal::read_rc(RcCommand& command) {
  if (!command_available_) {
    return false;
  }
  command = command_;
  return true;
}

void SimHal::write_motors(const MotorOutput& output) { motors_ = output; }

void SimHal::reset() {
  now_us_ = 0;
  acceleration_ = {};
  command_ = {};
  motors_ = {};
  acceleration_available_ = false;
  command_available_ = false;
}

void SimHal::set_now(Micros now_us) { now_us_ = now_us; }

void SimHal::publish_acceleration(const AccelerationSample& sample) {
  acceleration_ = sample;
  acceleration_available_ = true;
}

void SimHal::publish_rc(const RcCommand& command) {
  command_ = command;
  command_available_ = true;
}

const MotorOutput& SimHal::motor_output() const { return motors_; }

const AccelerationSample* SimHal::delivered_acceleration() const {
  return acceleration_available_ ? &acceleration_ : nullptr;
}

}  // namespace melty::sim
