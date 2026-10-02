#pragma once

#include <cstdint>

#include "melty/runtime.hpp"

namespace melty {

struct BoardProfile {
  std::uint8_t i2c_sda_pin{0};
  std::uint8_t i2c_scl_pin{0};
  std::uint8_t rc_spin_pin{0};
  std::uint8_t rc_translate_x_pin{0};
  std::uint8_t rc_translate_y_pin{0};
  std::uint8_t rc_arm_pin{0};
  std::uint8_t rc_phase_reset_pin{0};
  std::uint8_t esc_a_pin{0};
  std::uint8_t esc_b_pin{0};
  std::uint8_t enable_pin{0};
  std::uint16_t rc_min_us{1000};
  std::uint16_t rc_center_us{1500};
  std::uint16_t rc_max_us{2000};
  std::uint16_t esc_safe_us{1000};
  std::uint16_t esc_max_us{2000};
  std::uint8_t h3lis331dl_address{0x18};
  double acceleration_scale_mps2_per_lsb{0.47884}; // ±100 g, 12-bit left-justified.
  RuntimeConfig runtime{};
};

// These are reference wiring/calibration profiles, not robot-ready defaults.
bool valid_board_profile(const BoardProfile& profile);
BoardProfile esp32_devkit_reference_profile();
BoardProfile teensy41_reference_profile();

}  // namespace melty
