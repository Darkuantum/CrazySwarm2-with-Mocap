// uwb.h -- UWB two-way-ranging RESPONDER abstraction for the Qorvo DW3220.
//
// *** THIS IS AN INTERFACE + STUB, NOT A WORKING DRIVER. ***
//
// The band is the ranging RESPONDER: the drone (initiator) sends a poll
// message, this module replies, the drone computes time-of-flight and
// therefore distance. In a classic double-sided two-way-ranging (DS-TWR)
// exchange the responder still has to timestamp its own TX/RX and often
// echo a timestamp back to the initiator, so "just reply" is not quite
// "do nothing" -- see the TODOs below for exactly which vendor calls are
// needed and why.
//
// We deliberately do NOT invent detailed fake Qorvo API signatures beyond
// naming the real DW3000-family driver entry points (the `dwt_*` functions
// from Qorvo's DW3000/DW3xxx low-level driver, "DW3000 API" package) that
// a real implementation would call. Pull the actual vendor driver source
// (Qorvo's DW3XXX_Driver / dw3000-api) into `lib/` and fill in uwb.cpp
// against its real headers -- do not guess at struct layouts.
#pragma once

#include <stdint.h>

#include <Arduino.h>  // for IRAM_ATTR

namespace uwb {

// Pin assignment, fixed by the PCB. SPI bus pins are wired to the ESP32's
// default VSPI/HSPI-capable pins; CS/IRQ/RST are plain GPIOs.
//
// Deliberately a plain aggregate (no default member initializers, no
// user-declared constructor) so that `Pins{18, 19, 23, 5, 34, 27}`
// brace-initialization works under plain C++11, not just C++14-and-up --
// the board's actual defaults live on UwbResponder's constructor below
// instead of on this struct.
struct Pins {
  int sck;
  int miso;
  int mosi;
  int cs;
  int irq;  // DW3220 IRQ output -> ESP32 input, active high, edge-triggered
  int rst;  // DW3220 RSTn -> ESP32 output, active low, open-drain on the module
};

enum class RangingState : uint8_t {
  kNotInitialized,
  kIdle,           // initialized, waiting for a poll from the initiator
  kRangingActive,  // mid-exchange (poll seen, response scheduled/sent)
  kFault,          // SPI/IRQ/reset self-check failed; uwb_ok should read false
};

// A single completed ranging outcome, as reported by the drone-side
// initiator math -- the responder itself does not necessarily compute
// final distance (that can live on the initiator), but the field is here
// for a future firmware revision that also wants a local estimate, e.g.
// for on-band diagnostics/logging.
struct RangeResult {
  bool valid = false;
  float distance_m = 0.0f;
  uint32_t timestamp_ms = 0;
};

// UwbResponder: thin abstraction over the DW3220 so main.cpp never touches
// SPI registers directly. Every method is non-blocking except begin(),
// which does one-time bring-up and is allowed to take a few milliseconds.
class UwbResponder {
 public:
  explicit UwbResponder(const Pins& pins = Pins{18, 19, 23, 5, 34, 27});

  // One-time bring-up: configure SPI, reset the part, verify its device
  // ID, load the RF/UWB channel configuration, and arm RX so it's ready
  // to answer a poll. Returns true only if the part responded and
  // configuration succeeded. Safe to call once from setup().
  bool begin();

  // Call every loop() iteration. Non-blocking: services the IRQ line
  // (edge already latched by hardware; this polls a flag set by the ISR,
  // it does not itself block on SPI transactions taking arbitrarily
  // long) and advances the responder state machine.
  void poll();

  // Heartbeat's `uwb_ok` field: true once begin() succeeded and no fault
  // has been observed since (SPI timeout, unexpected IRQ pattern, etc).
  bool isHealthy() const { return state_ != RangingState::kFault && state_ != RangingState::kNotInitialized; }

  RangingState state() const { return state_; }

  // Most recent completed ranging result, if the responder-side firmware
  // is ever extended to compute one locally. Until then this simply never
  // becomes valid -- see TODOs in uwb.cpp.
  RangeResult lastResult() const { return lastResult_; }

  // ISR trampoline target for the DW3220 IRQ pin. Kept tiny and
  // IRAM-resident per ESP32 ISR conventions: it may ONLY set a flag, it
  // must never touch SPI (SPI transactions are not ISR-safe on this
  // core). The real interrupt servicing happens in poll().
  void IRAM_ATTR onIrq();

 private:
  Pins pins_;
  RangingState state_ = RangingState::kNotInitialized;
  RangeResult lastResult_;

  // Set by onIrq(), cleared by poll() once serviced. volatile: written
  // from ISR context, read from loop() context.
  volatile bool irqPending_ = false;

  // TODO(vendor-driver): once the Qorvo DW3000 API is vendored in, this
  // class will need somewhere to hold:
  //   - the dwt_config_t (channel, PRF, data rate, PAC size, preamble
  //     code/length, SFD type) applied in begin()
  //   - antenna delay calibration values (TX/RX) used to correct the
  //     timestamps this responder reports back to the initiator
  // Add those members here once integrating against the real headers
  // rather than guessing the struct shape now.
};

}  // namespace uwb
