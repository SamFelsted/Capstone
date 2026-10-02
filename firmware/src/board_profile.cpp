#include "melty/board_profile.hpp"

namespace melty {

BoardProfile esp32_devkit_reference_profile() {
  BoardProfile p{};
  p.i2c_sda_pin = 21;
  p.i2c_scl_pin = 22;
  p.rc_spin_pin = 32;
  p.rc_translate_x_pin = 33;
  p.rc_translate_y_pin = 25;
  p.rc_arm_pin = 26;
  p.rc_phase_reset_pin = 27;
  p.esc_a_pin = 18;
  p.esc_b_pin = 19;
  p.enable_pin = 23;
  return p;
}

BoardProfile teensy41_reference_profile() {
  BoardProfile p{};
  p.i2c_sda_pin = 18;
  p.i2c_scl_pin = 19;
  p.rc_spin_pin = 2;
  p.rc_translate_x_pin = 3;
  p.rc_translate_y_pin = 4;
  p.rc_arm_pin = 5;
  p.rc_phase_reset_pin = 6;
  p.esc_a_pin = 7;
  p.esc_b_pin = 8;
  p.enable_pin = 9;
  return p;
}

}  // namespace melty
