#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "melty/types.hpp"

namespace melty {

struct PwmRcConfig {
  std::uint32_t pulse_min_us{1000};
  std::uint32_t pulse_center_us{1500};
  std::uint32_t pulse_max_us{2000};
  std::uint32_t electrical_min_us{800};
  std::uint32_t electrical_max_us{2200};
  std::uint32_t channel_timeout_us{100000};
};

struct PwmRcCapture {
  static constexpr std::size_t channel_count = 5;
  std::array<std::uint32_t, channel_count> pulse_us{};
  std::array<std::uint32_t, channel_count> ended_us{};
  std::array<std::uint32_t, channel_count> sequence{};
  std::array<bool, channel_count> have_pulse{};
  std::uint32_t now_raw_us{0};
  Micros now_us{0};
  bool hardware_enabled{false};
};

// Converts asynchronously captured PWM channels into coherent command frames.
// A new timestamp is published only after every required channel has advanced
// since the prior publication. Partial updates continue returning the previous
// complete frame, with its original timestamp, until all channels advance.
class PwmRcFrameAssembler {
 public:
  explicit PwmRcFrameAssembler(const PwmRcConfig& config = {})
      : config_(config) {}

  void reset() {
    published_sequence_.fill(0);
    published_once_.fill(false);
    cached_ = {};
    cached_.hardware_enabled = false;
    have_frame_ = false;
  }

  bool update(const PwmRcCapture& capture, RcCommand& command) {
    if (!valid_config()) {
      return false;
    }

    bool all_advanced = true;
    Micros oldest = capture.now_us;
    for (std::size_t i = 0; i < PwmRcCapture::channel_count; ++i) {
      const std::uint32_t age = capture.now_raw_us - capture.ended_us[i];
      if (!capture.have_pulse[i] ||
          capture.pulse_us[i] < config_.electrical_min_us ||
          capture.pulse_us[i] > config_.electrical_max_us ||
          age > config_.channel_timeout_us || age > capture.now_us) {
        return false;
      }
      oldest = std::min(oldest, capture.now_us - age);
      all_advanced = all_advanced &&
                     (!published_once_[i] ||
                      capture.sequence[i] != published_sequence_[i]);
    }

    if (all_advanced) {
      cached_.spin = normalized_unipolar(capture.pulse_us[0]);
      cached_.translate_x = normalized_centered(capture.pulse_us[1]);
      cached_.translate_y = normalized_centered(capture.pulse_us[2]);
      cached_.arm = capture.pulse_us[3] > config_.pulse_center_us;
      cached_.reset_phase =
          capture.pulse_us[4] > config_.pulse_center_us;
      cached_.timestamp_us = oldest;
      published_sequence_ = capture.sequence;
      published_once_.fill(true);
      have_frame_ = true;
    }

    if (!have_frame_) {
      return false;
    }
    cached_.hardware_enabled = capture.hardware_enabled;
    command = cached_;
    return true;
  }

 private:
  bool valid_config() const {
    return config_.electrical_min_us <= config_.pulse_min_us &&
           config_.pulse_min_us < config_.pulse_center_us &&
           config_.pulse_center_us < config_.pulse_max_us &&
           config_.pulse_max_us <= config_.electrical_max_us &&
           config_.channel_timeout_us > 0;
  }

  double normalized_unipolar(std::uint32_t pulse_us) const {
    const double span = config_.pulse_max_us - config_.pulse_min_us;
    const double value =
        (static_cast<double>(pulse_us) - config_.pulse_min_us) / span;
    return std::max(0.0, std::min(1.0, value));
  }

  double normalized_centered(std::uint32_t pulse_us) const {
    if (pulse_us >= config_.pulse_center_us) {
      const double span = config_.pulse_max_us - config_.pulse_center_us;
      return std::min(
          1.0, (pulse_us - config_.pulse_center_us) / span);
    }
    const double span = config_.pulse_center_us - config_.pulse_min_us;
    return std::max(
        -1.0,
        -static_cast<double>(config_.pulse_center_us - pulse_us) / span);
  }

  PwmRcConfig config_{};
  std::array<std::uint32_t, PwmRcCapture::channel_count> published_sequence_{};
  std::array<bool, PwmRcCapture::channel_count> published_once_{};
  RcCommand cached_{};
  bool have_frame_{false};
};

}  // namespace melty
