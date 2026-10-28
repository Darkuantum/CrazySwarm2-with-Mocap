// haptics.h -- non-blocking vibration-motor feedback patterns.
//
// Motor is driven through a transistor (GPIO25 -> base/gate), PWM'd via
// the ESP32 LEDC peripheral so intensity is adjustable and the drive
// signal is a clean fixed-frequency PWM rather than a bit-banged digital
// on/off. Everything here is timer-polled from loop() -- no delay(), so
// a haptic pattern playing never blocks gesture sampling, BLE heartbeats,
// or (most importantly) E-stop handling.
#pragma once

#include <stdint.h>

namespace haptics {

// Pattern IDs match the values the drone writes to the haptic (write)
// BLE characteristic 1:1 -- do not renumber without updating the drone
// side and README.
enum PatternId : uint8_t {
  kPatternNone = 0,
  kPatternShortBlip = 1,   // command acknowledged
  kPatternDoublePulse = 2, // target lost
  kPatternLongBuzz = 3,    // failsafe / hold entered
};

// Plain aggregate (no default member initializer, no user-declared
// constructor) so brace-init works under plain C++11 -- see the same note
// on uwb::Pins. The board's actual default pin lives on HapticDriver's
// constructor below.
struct Pins {
  int motor;
};

class HapticDriver {
 public:
  explicit HapticDriver(const Pins& pins = Pins{25});

  // One-time LEDC setup. Call once from setup().
  void begin();

  // Starts playing `pattern` from its first step, interrupting whatever
  // was previously playing (last-write-wins -- matches "the drone tells
  // us what's happening now", not a queue of past events).
  void trigger(PatternId pattern);

  // Advances the current pattern based on elapsed time. Call every
  // loop() iteration; cheap (a couple of comparisons) when idle.
  void poll();

  bool isPlaying() const { return pattern_ != kPatternNone; }

 private:
  struct Step {
    uint8_t dutyPercent;  // 0..100; 0 = motor off for this step
    uint16_t durationMs;
  };

  Pins pins_;
  int ledcChannel_ = 0;

  PatternId pattern_ = kPatternNone;
  uint8_t stepIndex_ = 0;
  uint32_t stepStartMs_ = 0;

  void enterStep(uint8_t index);
  void applyDuty(uint8_t dutyPercent);

  // Returns the step table for `pattern` and its length, or nullptr/0 if
  // `pattern` has no defined sequence (e.g. kPatternNone).
  static const Step* stepsFor(PatternId pattern, uint8_t* outLen);
};

}  // namespace haptics
