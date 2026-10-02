#if defined(ARDUINO)

#include <Arduino.h>

#include "arduino_hal.hpp"
#include "melty/board_profile.hpp"
#include "melty/runtime.hpp"

namespace {

#if defined(ARDUINO_ARCH_ESP32)
melty::BoardProfile profile = melty::esp32_devkit_reference_profile();
#elif defined(ARDUINO_TEENSY41)
melty::BoardProfile profile = melty::teensy41_reference_profile();
#else
#error "No reference board profile for this Arduino target"
#endif

melty::ArduinoReferenceHal hardware(profile);
melty::Runtime* runtime = nullptr;
melty::Micros next_tick_us = 0;

}  // namespace

void setup() {
  const bool hardware_ok = hardware.begin();
#if defined(MELTY_ENABLE_REFERENCE_HARDWARE)
  if (hardware_ok) {
    runtime = new melty::Runtime(hardware, profile.runtime);
    next_tick_us = hardware.now_us();
  }
#else
  (void)hardware_ok;
#endif
}

void loop() {
  if (runtime == nullptr) {
    hardware.write_motors({});
    delay(10);
    return;
  }
  const melty::Micros now = hardware.now_us();
  if (now >= next_tick_us) {
    runtime->tick();
    next_tick_us += profile.runtime.control_period_us;
    if (now > next_tick_us + profile.runtime.maximum_tick_interval_us) {
      next_tick_us = now + profile.runtime.control_period_us;
    }
  }
}

#endif  // ARDUINO
