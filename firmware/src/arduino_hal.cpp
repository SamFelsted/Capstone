#include "arduino_hal.hpp"

#if defined(ARDUINO)

#include <algorithm>
#include <cmath>

namespace melty {
namespace {

constexpr std::uint8_t kWhoAmI = 0x0f;
constexpr std::uint8_t kWhoAmIExpected = 0x32;
constexpr std::uint8_t kControl1 = 0x20;
constexpr std::uint8_t kControl4 = 0x23;
constexpr std::uint8_t kStatus = 0x27;
constexpr std::uint8_t kOutputXLow = 0x28;
constexpr std::uint32_t kRcElectricalMinUs = 800;
constexpr std::uint32_t kRcElectricalMaxUs = 2200;
constexpr std::uint32_t kRcCaptureTimeoutUs = 100000;

std::uint16_t clamp_pulse(long value, std::uint16_t low, std::uint16_t high) {
  return static_cast<std::uint16_t>(std::max<long>(low, std::min<long>(value, high)));
}

int integer_magnitude(int value) { return value < 0 ? -value : value; }

}  // namespace

ArduinoReferenceHal* ArduinoReferenceHal::instance_ = nullptr;

ArduinoReferenceHal::ArduinoReferenceHal(const BoardProfile& profile)
    : profile_(profile) {}

bool ArduinoReferenceHal::begin() {
  ready_ = false;
  instance_ = this;
  pinMode(profile_.enable_pin, INPUT_PULLDOWN);
  pinMode(profile_.esc_a_pin, OUTPUT);
  pinMode(profile_.esc_b_pin, OUTPUT);
  configure_pwm();
  write_esc(profile_.esc_a_pin, 0, profile_.esc_safe_us);
  write_esc(profile_.esc_b_pin, 1, profile_.esc_safe_us);

  const std::uint8_t rc_pins[5] = {
      profile_.rc_spin_pin, profile_.rc_translate_x_pin,
      profile_.rc_translate_y_pin, profile_.rc_arm_pin,
      profile_.rc_phase_reset_pin};
  void (*isrs[5])() = {rc_isr_0, rc_isr_1, rc_isr_2, rc_isr_3, rc_isr_4};
  for (unsigned i = 0; i < 5; ++i) {
    pinMode(rc_pins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(rc_pins[i]), isrs[i], CHANGE);
  }

#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(profile_.i2c_sda_pin, profile_.i2c_scl_pin);
#elif defined(ARDUINO_TEENSY41)
  // The reference profile uses Teensy 4.1's fixed Wire pins 18/19.
  Wire.begin();
#endif
  Wire.setClock(400000);
  std::uint8_t identity = 0;
  if (!read_register(kWhoAmI, identity) || identity != kWhoAmIExpected) {
    return false;
  }
  // Normal mode, 1000 Hz output data rate, all axes enabled.
  if (!write_register(kControl1, 0x3f)) {
    return false;
  }
  // Block data update, little-endian, +/-100 g.
  if (!write_register(kControl4, 0x80)) {
    return false;
  }
  ready_ = true;
  return true;
}

Micros ArduinoReferenceHal::now_us() {
  noInterrupts();
  const std::uint32_t raw = micros();
  if (raw < last_micros_raw_) {
    micros_high_ += (std::uint64_t{1} << 32);
  }
  last_micros_raw_ = raw;
  const Micros extended = micros_high_ | raw;
  interrupts();
  return extended;
}

bool ArduinoReferenceHal::write_register(std::uint8_t reg, std::uint8_t value) {
  Wire.beginTransmission(profile_.h3lis331dl_address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool ArduinoReferenceHal::read_register(std::uint8_t reg, std::uint8_t& value) {
  Wire.beginTransmission(profile_.h3lis331dl_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(profile_.h3lis331dl_address, std::uint8_t{1}) != 1) {
    return false;
  }
  value = static_cast<std::uint8_t>(Wire.read());
  return true;
}

bool ArduinoReferenceHal::read_acceleration_registers() {
  Wire.beginTransmission(profile_.h3lis331dl_address);
  Wire.write(static_cast<std::uint8_t>(kOutputXLow | 0x80));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(profile_.h3lis331dl_address, std::uint8_t{6}) != 6) {
    return false;
  }
  const std::uint8_t xl = Wire.read();
  const std::uint8_t xh = Wire.read();
  const std::uint8_t yl = Wire.read();
  const std::uint8_t yh = Wire.read();
  (void)Wire.read();
  (void)Wire.read();
  const std::int16_t x16 = static_cast<std::int16_t>(
      static_cast<std::uint16_t>(xl) | (static_cast<std::uint16_t>(xh) << 8));
  const std::int16_t y16 = static_cast<std::int16_t>(
      static_cast<std::uint16_t>(yl) | (static_cast<std::uint16_t>(yh) << 8));
  latest_acceleration_.x_mps2 =
      -static_cast<double>(x16 >> 4) * profile_.acceleration_scale_mps2_per_lsb;
  latest_acceleration_.y_mps2 =
      -static_cast<double>(y16 >> 4) * profile_.acceleration_scale_mps2_per_lsb;
  latest_acceleration_.saturated =
      integer_magnitude(static_cast<int>(x16 >> 4)) >= 2040 ||
      integer_magnitude(static_cast<int>(y16 >> 4)) >= 2040;
  latest_acceleration_.timestamp_us = now_us();
  have_acceleration_ = true;
  return true;
}

bool ArduinoReferenceHal::read_acceleration(AccelerationSample& sample) {
  if (!ready_) {
    return false;
  }
  std::uint8_t status = 0;
  if (!read_register(kStatus, status)) {
    return false;
  }
  if ((status & 0x08u) != 0 && !read_acceleration_registers()) {
    return false;
  }
  if (!have_acceleration_) {
    return false;
  }
  sample = latest_acceleration_;
  return true;
}

void ArduinoReferenceHal::on_rc_edge(unsigned channel) {
  const std::uint8_t pins[5] = {
      profile_.rc_spin_pin, profile_.rc_translate_x_pin,
      profile_.rc_translate_y_pin, profile_.rc_arm_pin,
      profile_.rc_phase_reset_pin};
  const std::uint32_t now = micros();
  if (digitalRead(pins[channel]) != LOW) {
    rc_rise_us_[channel] = now;
  } else {
    const std::uint32_t width = now - rc_rise_us_[channel];
    if (width >= kRcElectricalMinUs && width <= kRcElectricalMaxUs) {
      rc_pulse_us_[channel] = width;
      rc_end_us_[channel] = now;
    }
  }
}

void ArduinoReferenceHal::rc_isr_0() { instance_->on_rc_edge(0); }
void ArduinoReferenceHal::rc_isr_1() { instance_->on_rc_edge(1); }
void ArduinoReferenceHal::rc_isr_2() { instance_->on_rc_edge(2); }
void ArduinoReferenceHal::rc_isr_3() { instance_->on_rc_edge(3); }
void ArduinoReferenceHal::rc_isr_4() { instance_->on_rc_edge(4); }

double ArduinoReferenceHal::normalized_unipolar(std::uint32_t pulse_us) const {
  const double span = profile_.rc_max_us - profile_.rc_min_us;
  const double value = (static_cast<double>(pulse_us) - profile_.rc_min_us) / span;
  return std::max(0.0, std::min(1.0, value));
}

double ArduinoReferenceHal::normalized_centered(std::uint32_t pulse_us) const {
  if (pulse_us >= profile_.rc_center_us) {
    const double span = profile_.rc_max_us - profile_.rc_center_us;
    return std::min(1.0, (pulse_us - profile_.rc_center_us) / span);
  }
  const double span = profile_.rc_center_us - profile_.rc_min_us;
  return std::max(-1.0, -static_cast<double>(profile_.rc_center_us - pulse_us) / span);
}

bool ArduinoReferenceHal::read_rc(RcCommand& command) {
  if (!ready_) {
    return false;
  }
  std::uint32_t pulse[5]{};
  std::uint32_t ended[5]{};
  noInterrupts();
  for (unsigned i = 0; i < 5; ++i) {
    pulse[i] = rc_pulse_us_[i];
    ended[i] = rc_end_us_[i];
  }
  interrupts();

  const Micros now = now_us();
  const std::uint32_t now_raw = static_cast<std::uint32_t>(now);
  Micros oldest = now;
  for (unsigned i = 0; i < 5; ++i) {
    const std::uint32_t age = now_raw - ended[i];
    if (pulse[i] < kRcElectricalMinUs || pulse[i] > kRcElectricalMaxUs ||
        ended[i] == 0 || age > kRcCaptureTimeoutUs) {
      return false;
    }
    oldest = std::min(oldest, now - age);
  }
  command.spin = normalized_unipolar(pulse[0]);
  command.translate_x = normalized_centered(pulse[1]);
  command.translate_y = normalized_centered(pulse[2]);
  command.arm = pulse[3] > profile_.rc_center_us &&
                digitalRead(profile_.enable_pin) == HIGH;
  command.reset_phase = pulse[4] > profile_.rc_center_us;
  command.timestamp_us = oldest;
  return true;
}

void ArduinoReferenceHal::configure_pwm() {
#if defined(ARDUINO_ARCH_ESP32)
  ledcSetup(0, 50, 16);
  ledcSetup(1, 50, 16);
  ledcAttachPin(profile_.esc_a_pin, 0);
  ledcAttachPin(profile_.esc_b_pin, 1);
#elif defined(ARDUINO_TEENSY41)
  analogWriteResolution(16);
  analogWriteFrequency(profile_.esc_a_pin, 50);
  analogWriteFrequency(profile_.esc_b_pin, 50);
#endif
}

void ArduinoReferenceHal::write_esc(std::uint8_t pin, unsigned channel,
                                    std::uint16_t pulse_us) {
  const std::uint32_t duty =
      (static_cast<std::uint32_t>(pulse_us) * 65535u) / 20000u;
#if defined(ARDUINO_ARCH_ESP32)
  (void)pin;
  ledcWrite(channel, duty);
#elif defined(ARDUINO_TEENSY41)
  (void)channel;
  analogWrite(pin, duty);
#endif
}

void ArduinoReferenceHal::write_motors(const MotorOutput& output) {
  const bool hardware_enabled = ready_ && digitalRead(profile_.enable_pin) == HIGH;
  const double a = hardware_enabled && std::isfinite(output.wheel_a)
                       ? std::max(0.0, std::min(1.0, output.wheel_a))
                       : 0.0;
  const double b = hardware_enabled && std::isfinite(output.wheel_b)
                       ? std::max(0.0, std::min(1.0, output.wheel_b))
                       : 0.0;
  const long span = profile_.esc_max_us - profile_.esc_safe_us;
  write_esc(profile_.esc_a_pin, 0,
            clamp_pulse(profile_.esc_safe_us + lround(a * span),
                        profile_.esc_safe_us, profile_.esc_max_us));
  write_esc(profile_.esc_b_pin, 1,
            clamp_pulse(profile_.esc_safe_us + lround(b * span),
                        profile_.esc_safe_us, profile_.esc_max_us));
}

}  // namespace melty

#endif  // ARDUINO
