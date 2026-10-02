#pragma once

#if defined(ARDUINO)

#include <Arduino.h>
#include <Wire.h>

#include "melty/board_profile.hpp"
#include "melty/hal.hpp"

namespace melty {

class ArduinoReferenceHal final : public Hal {
 public:
  explicit ArduinoReferenceHal(const BoardProfile& profile);

  bool begin();
  Micros now_us() override;
  bool read_acceleration(AccelerationSample& sample) override;
  bool read_rc(RcCommand& command) override;
  void write_motors(const MotorOutput& output) override;

 private:
  static ArduinoReferenceHal* instance_;
  static void rc_isr_0();
  static void rc_isr_1();
  static void rc_isr_2();
  static void rc_isr_3();
  static void rc_isr_4();
  void on_rc_edge(unsigned channel);

  bool write_register(std::uint8_t reg, std::uint8_t value);
  bool read_register(std::uint8_t reg, std::uint8_t& value);
  bool read_acceleration_registers();
  void configure_pwm();
  void write_esc(std::uint8_t pin, unsigned channel, std::uint16_t pulse_us);
  double normalized_unipolar(std::uint32_t pulse_us) const;
  double normalized_centered(std::uint32_t pulse_us) const;

  BoardProfile profile_{};
  bool ready_{false};
  bool have_acceleration_{false};
  std::uint32_t last_micros_raw_{0};
  std::uint64_t micros_high_{0};
  AccelerationSample latest_acceleration_{};

  volatile std::uint32_t rc_rise_us_[5]{};
  volatile std::uint32_t rc_pulse_us_[5]{};
  volatile std::uint32_t rc_end_us_[5]{};
};

}  // namespace melty

#endif  // ARDUINO
