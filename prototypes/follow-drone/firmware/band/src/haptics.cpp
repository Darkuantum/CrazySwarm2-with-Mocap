#include "haptics.h"

#include <Arduino.h>

namespace haptics {

namespace {
constexpr int kLedcFreqHz = 20000;  // above audible range; motor is ERM, not resonant
constexpr int kLedcResolutionBits = 8;  // duty 0..255
constexpr uint32_t kLedcMaxDuty = (1u << kLedcResolutionBits) - 1;

// Pattern step tables. Kept as static const data (flash, not RAM) --
// this is the only place pattern shapes are defined; tune feel here.
const HapticDriver::Step kShortBlip[] = {
    {100, 120},  // one firm blip
};
const HapticDriver::Step kDoublePulse[] = {
    {100, 80},
    {0, 80},
    {100, 80},
};
const HapticDriver::Step kLongBuzz[] = {
    {80, 600},  // slightly softer duty than a blip so a 600ms buzz isn't jarring
};
}  // namespace

const HapticDriver::Step* HapticDriver::stepsFor(PatternId pattern, uint8_t* outLen) {
  switch (pattern) {
    case kPatternShortBlip:
      *outLen = sizeof(kShortBlip) / sizeof(kShortBlip[0]);
      return kShortBlip;
    case kPatternDoublePulse:
      *outLen = sizeof(kDoublePulse) / sizeof(kDoublePulse[0]);
      return kDoublePulse;
    case kPatternLongBuzz:
      *outLen = sizeof(kLongBuzz) / sizeof(kLongBuzz[0]);
      return kLongBuzz;
    case kPatternNone:
    default:
      *outLen = 0;
      return nullptr;
  }
}

HapticDriver::HapticDriver(const Pins& pins) : pins_(pins) {}

void HapticDriver::begin() {
  // NOTE: this targets the "classic" arduino-esp32 core v2.x LEDC API
  // (ledcSetup/ledcAttachPin/ledcWrite by channel number). If this
  // project is built against arduino-esp32 core v3.x, the LEDC API moved
  // to ledcAttach(pin, freq, resBits)/ledcWrite(pin, duty) (no explicit
  // channel argument) -- swap these three calls accordingly; the rest of
  // this file (the polling state machine) is unaffected either way.
  ledcSetup(ledcChannel_, kLedcFreqHz, kLedcResolutionBits);
  ledcAttachPin(pins_.motor, ledcChannel_);
  ledcWrite(ledcChannel_, 0);
}

void HapticDriver::applyDuty(uint8_t dutyPercent) {
  const uint32_t duty = (kLedcMaxDuty * dutyPercent) / 100;
  ledcWrite(ledcChannel_, duty);
}

void HapticDriver::enterStep(uint8_t index) {
  uint8_t len = 0;
  const Step* steps = stepsFor(pattern_, &len);
  if (steps == nullptr || index >= len) {
    // Pattern finished (or invalid): motor off, return to idle.
    applyDuty(0);
    pattern_ = kPatternNone;
    stepIndex_ = 0;
    return;
  }
  stepIndex_ = index;
  stepStartMs_ = millis();
  applyDuty(steps[index].dutyPercent);
}

void HapticDriver::trigger(PatternId pattern) {
  pattern_ = pattern;
  enterStep(0);
}

void HapticDriver::poll() {
  if (pattern_ == kPatternNone) return;

  uint8_t len = 0;
  const Step* steps = stepsFor(pattern_, &len);
  if (steps == nullptr || stepIndex_ >= len) {
    enterStep(len);  // out-of-range on purpose -> finishes/cleans up
    return;
  }

  if (millis() - stepStartMs_ >= steps[stepIndex_].durationMs) {
    enterStep(stepIndex_ + 1);
  }
}

}  // namespace haptics
