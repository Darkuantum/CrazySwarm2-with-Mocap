// uwb.cpp -- STUBBED DW3220 driver. See the big warning in uwb.h.
//
// Nothing in this file talks to real DW3220 silicon yet. It exists so
// main.cpp, the heartbeat's uwb_ok field, and the overall program
// structure are all wired up correctly ahead of the vendor driver
// integration, which is a separate, larger piece of work (bringing in
// Qorvo's DW3000 API / DW3XXX driver package, licensing it into this
// repo, and validating SPI timing against the DW3220 datasheet).
//
// ---------------------------------------------------------------------
// TODO(vendor-driver): replace this file's guts with real calls into
// Qorvo's DW3000-family low-level driver ("DW3000 API", `dwt_*` entry
// points). At minimum, a real responder implementation needs:
//
//   1. SPI + reset bring-up
//      - Configure the ESP32 SPI peripheral (SPIClass::begin(SCK, MISO,
//        MOSI, CS)) at the clock the DW3220 datasheet allows for
//        register access before/after PLL lock (slower before, faster
//        after -- the vendor driver typically exposes a "slow"/"fast"
//        SPI rate switch, e.g. port_set_dw_ics()/similar in the
//        reference platform layer).
//      - Toggle RSTn (our `rst` pin) per the datasheet reset pulse width,
//        then wait for the SPIRDY/IRQ line per the vendor driver's boot
//        sequence.
//      - dwt_initialise(...) or dwt_probe(...) (naming differs slightly
//        across DW3000-family driver versions) to bring up the part and
//        confirm chip ID/PDOA capability.
//
//   2. Configuration
//      - dwt_configure(&config) with a dwt_config_t describing channel,
//        PRF, preamble length/code, data rate, PAC size, SFD type -- this
//        MUST match the initiator's (drone-side) configuration exactly or
//        ranging will simply never see a valid preamble.
//      - dwt_setrxantennadelay(...) / dwt_settxantennadelay(...) with
//        antenna delay values from range calibration against a known
//        distance -- these directly bias every reported range and are
//        typically the single biggest source of systematic ranging
//        error if left at defaults.
//      - dwt_setleds(...) optionally, for a status LED tied to a DW3220
//        GPIO if the PCB wires one up.
//
//   3. Interrupt + callback wiring
//      - attachInterrupt(pins_.irq, isrTrampoline, RISING) on the ESP32
//        side (the DW3220 IRQ line is active-high, push-pull, edge -- the
//        trampoline here only sets a flag; see onIrq()).
//      - dwt_setcallbacks(&cbTxDone, &cbRxOk, &cbRxTimeout, &cbRxError)
//        (or the DW3000-driver-version-appropriate equivalent) so
//        poll() -> dwt_isr() dispatches into responder logic.
//      - dwt_setinterrupt(...) to select which internal DW3220 events
//        actually assert the IRQ pin (RX good frame, RX timeout, RX
//        error, TX done -- a responder needs at least RX-good and
//        TX-done).
//
//   4. The actual DS-TWR responder exchange
//      - dwt_rxenable(DWT_START_RX_IMMEDIATE) to arm the receiver while
//        idle, waiting for the initiator's "Poll" message.
//      - On RX-good with a recognized Poll message: read the poll's
//        embedded initiator TX timestamp, then dwt_setdelayedtrxtime(...)
//        + dwt_starttx(DWT_START_TX_DELAYED) to send the "Response"
//        message at a precisely scheduled time (DS-TWR needs known,
//        fixed turnaround delays on both ends).
//      - dwt_readrxtimestamp(...) / dwt_readtxtimestamp(...) to capture
//        this side's own RX/TX timestamps, which get embedded in the
//        Response (and/or a follow-up Report) message so the initiator
//        can complete the DS-TWR time-of-flight computation.
//      - Frame construction/CRC is handled by the DW3220 itself
//        (dwt_writetxdata / dwt_writetxfctrl) -- do not hand-roll 802.15.4
//        framing.
//
//   5. Fault handling
//      - dwt_rxenable() can leave the part in an error state on repeated
//        RX errors/timeouts; the vendor driver typically wants an
//        explicit dwt_forcetrxoff() + dwt_rxenable() recovery cycle. Feed
//        that into state_ = RangingState::kFault -> retry -> kIdle so
//        isHealthy() (and therefore the heartbeat's uwb_ok bit) reflects
//        reality instead of latching healthy forever after one good boot.
//
// None of the above is implemented below. begin() honestly reports
// failure-to-detect-hardware today (state_ stays kNotInitialized) rather
// than pretending to succeed, so uwb_ok in the heartbeat will read false
// until this is done -- that's intentional, not a bug.
// ---------------------------------------------------------------------

#include "uwb.h"

namespace uwb {

namespace {
// A single instance is expected (one DW3220 on the board); the ISR
// trampoline needs a plain-C-function target, so it reaches back into
// this instance. If a second UwbResponder is ever needed, this pattern
// needs to grow into a small registry -- not worth it for one radio.
UwbResponder* g_activeInstance = nullptr;

void IRAM_ATTR isrTrampoline() {
  if (g_activeInstance != nullptr) {
    g_activeInstance->onIrq();
  }
}
}  // namespace

UwbResponder::UwbResponder(const Pins& pins) : pins_(pins) {}

bool UwbResponder::begin() {
  // TODO(vendor-driver): SPI.begin(pins_.sck, pins_.miso, pins_.mosi,
  // pins_.cs); pulse pins_.rst; dwt_initialise(...); dwt_configure(...);
  // dwt_setrxantennadelay(...); dwt_settxantennadelay(...);
  // attachInterrupt(pins_.irq, isrTrampoline, RISING); dwt_rxenable(...).
  //
  // Until that landing, we deliberately do NOT claim success: pinMode()
  // the control lines so they're in a known state (reset held/released
  // sanely, no floating inputs) and report "not initialized" rather than
  // fabricate a healthy responder.
  pinMode(pins_.rst, OUTPUT);
  digitalWrite(pins_.rst, HIGH);  // RSTn inactive; real bring-up pulses this low
  pinMode(pins_.irq, INPUT);
  pinMode(pins_.cs, OUTPUT);
  digitalWrite(pins_.cs, HIGH);  // CS idle-high (active low, unselected)

  g_activeInstance = this;

  // This part of the bring-up IS real (plain ESP32 GPIO interrupt
  // plumbing, no Qorvo driver involved): arm the IRQ line now so
  // irqPending_ starts working the moment the vendor driver is dropped
  // in and actually drives state_ past kNotInitialized. Attaching it
  // early and unconditionally is harmless -- with no vendor init done,
  // the DW3220 IRQ pin simply never asserts, so this never fires.
  attachInterrupt(digitalPinToInterrupt(pins_.irq), isrTrampoline, RISING);

  state_ = RangingState::kNotInitialized;
  return false;  // flip to true once dwt_initialise()/dwt_configure() land
}

void UwbResponder::poll() {
  if (state_ == RangingState::kNotInitialized) {
    return;  // nothing to service until begin() is a real driver call
  }

  if (irqPending_) {
    irqPending_ = false;
    // TODO(vendor-driver): dwt_isr() dispatch here, which fans out into
    // the cbRxOk/cbTxDone/cbRxTimeout/cbRxError callbacks registered in
    // begin(), driving state_ through kRangingActive and populating
    // lastResult_ when a full DS-TWR exchange completes.
  }
}

void IRAM_ATTR UwbResponder::onIrq() {
  // Deliberately minimal: ISR context on the ESP32 cannot safely perform
  // an SPI transaction against the DW3220, so all we do is note that
  // something happened and let poll() (regular task context) do the
  // real work via dwt_isr().
  irqPending_ = true;
}

}  // namespace uwb
