#include "melty/board_profile.hpp"

#include <array>
#include <cmath>

namespace melty {

bool valid_board_profile(const BoardProfile& p) {
  if (!(p.rc_min_us >= 750 && p.rc_min_us < p.rc_center_us &&
        p.rc_center_us < p.rc_max_us && p.rc_max_us <= 2250 &&
        p.esc_safe_us >= 750 && p.esc_safe_us < p.esc_max_us &&
        p.esc_max_us <= 2250 &&
        p.h3lis331dl_address >= 0x08 && p.h3lis331dl_address <= 0x77 &&
        std::isfinite(p.acceleration_scale_mps2_per_lsb) &&
        p.acceleration_scale_mps2_per_lsb > 0.0 &&
        Runtime::valid_config(p.runtime))) {
    return false;
  }

  const std::array<std::uint8_t, 10> pins = {
      p.i2c_sda_pin, p.i2c_scl_pin, p.rc_spin_pin, p.rc_translate_x_pin,
      p.rc_translate_y_pin, p.rc_arm_pin, p.rc_phase_reset_pin,
      p.esc_a_pin, p.esc_b_pin, p.enable_pin};
  for (std::size_t i = 0; i < pins.size(); ++i) {
    for (std::size_t j = i + 1; j < pins.size(); ++j) {
      if (pins[i] == pins[j]) {
        return false;
      }
    }
  }
  return true;
}

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
  p.runtime.controller.maximum_spin_rad_s = 150.0;
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
  p.runtime.controller.maximum_spin_rad_s = 150.0;
  return p;
}

}  // namespace melty
