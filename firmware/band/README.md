# Follow-Drone Wearable Band Firmware

Gesture + ranging wristband for a person-following drone. ESP32, Arduino
framework, PlatformIO. Talks to the drone over BLE.

**Build status: UNBUILT.** This firmware was written without PlatformIO
installed in the authoring environment, so it has never been compiled,
flashed, or run. Treat every claim below about behavior as a design intent
until it has been verified on real hardware. See "What is stubbed and
untested" at the bottom before you trust any of it near a person or a
flying drone.

## Pinout

| Signal              | ESP32 GPIO | Notes                                          |
|---------------------|-----------:|-------------------------------------------------|
| IMU I2C SDA          | 21         | BNO085, shared bus                              |
| IMU I2C SCL          | 22         | BNO085                                          |
| UWB SPI SCK          | 18         | DW3220                                          |
| UWB SPI MISO         | 19         | DW3220                                          |
| UWB SPI MOSI         | 23         | DW3220                                          |
| UWB SPI CS           | 5          | DW3220, active low                              |
| UWB IRQ              | 34         | DW3220 -> ESP32 input, active high, input-only pin |
| UWB RSTn             | 27         | ESP32 -> DW3220, active low                     |
| E-stop button        | 33         | INPUT_PULLUP, active low, input-only pin        |
| Vibration motor      | 25         | via transistor, LEDC PWM                        |
| Battery sense        | 35         | through resistor divider, input-only pin (ADC1) |

Notes:
- GPIO 34, 35 and 33 are input-only on the ESP32 (no internal pull
  capability on 34/35). The E-stop button on 33 does have an internal
  pull, which is why it's configured `INPUT_PULLUP` in software; 34/35 are
  driven externally (DW3220 IRQ, battery divider) so that's fine.
- The battery divider ratio assumed in `main.cpp` (`kBatteryDividerRatio`,
  currently 2.0, i.e. two equal-value resistors) MUST match the resistors
  actually populated on your board, or `battery_mv` in the heartbeat will
  be wrong by a constant factor. Fix the constant, don't add a runtime
  calibration knob for this -- it's a fixed property of the PCB.

## Building and flashing

```
cd firmware/band
pio run                       # build
pio run -t upload             # flash (board must be in bootloader mode)
pio device monitor             # serial monitor @ 115200
```

This has not been run. Things to check on your first attempt:
- `platformio.ini` lists `adafruit/Adafruit BNO08x`, `Adafruit BusIO`, and
  `Adafruit Unified Sensor` as `lib_deps`. Pin these to whatever the
  PlatformIO registry currently resolves if the `^1.x` ranges here have
  drifted.
- The ESP32 BLE Arduino API (`BLEDevice.h` etc.) ships inside the
  `arduino-esp32` core itself, not as a separate library -- if your
  installed core version doesn't bundle it (very old cores, or a core
  configured with NimBLE-only), you'll need to either upgrade the core or
  add `h2zero/NimBLE-Arduino` and adapt `main.cpp`'s BLE calls, which are
  written against the classic (Bluedroid-based) API's class names
  (`BLEServer`, `BLECharacteristic`, `BLE2902`, ...).
- `src/haptics.cpp` uses the "classic" `ledcSetup`/`ledcAttachPin`/`ledcWrite`
  LEDC API (arduino-esp32 core v2.x). If PlatformIO resolves core v3.x,
  that API changed to `ledcAttach(pin, freq, resBits)` / `ledcWrite(pin,
  duty)` (no explicit channel argument) -- there's a comment at the top of
  `HapticDriver::begin()` marking exactly what to change.

## Calibrating the gesture thresholds

All gesture-recognition tuning constants live in one place:
`gestures::GestureCalibration`, at the top of `src/gestures.h`. Nothing
else in the recognizer has a hidden magic number -- if you need to retune
against real operators, this struct is the entire surface area.

Suggested calibration procedure:

1. Add a temporary debug branch (or a serial print in
   `handleGestureResult()` in `main.cpp`) that logs every completed
   segment's features -- not just ones that passed the confidence
   threshold. The cleanest way is to give `GestureRecognizer::update()` an
   optional "always report, even below threshold" mode, or simply
   temporarily set `confidenceThreshold = 0` so every segment reaches
   `handleGestureResult()`. Log peak axis, peak magnitude, duration, zero
   crossings, and integrated yaw for each segment.
2. Have a real operator perform each of the five gestures 10-20 times at
   a natural pace, with natural idle motion in between (walking, arm
   swinging) so you also capture what should NOT trigger.
3. From the logged features:
   - `startThresholdMps2` / `endThresholdMps2`: set from the actual noise
     floor of idle arm motion vs. the smallest intentional gesture's peak
     -- you want clean separation with headroom, not the midpoint of the
     two observed distributions.
   - Per-gesture duration/peak/zero-crossing bounds (`stopMax...`,
     `landMin...`, `followMin...`, `comeMin...`, `orbitMin...`): set from
     the actual observed ranges for that gesture, with margin, not
     guessed values (the ones checked into this file ARE guessed values
     -- see the big comment block above the struct).
   - `confidenceThreshold`: raise until false positives during idle
     motion/other gestures stop, then back off slightly for margin.
   - `perGestureCooldownMs`: set to slightly longer than the fastest a
     real operator naturally repeats a deliberate gesture.
4. Re-run with the real thresholds and confirm both directions: every
   intended gesture fires with confidence above threshold, and normal arm
   motion (walking, adjusting the band, scratching your nose) does not
   spuriously fire anything.
5. `kForwardAxis`/`kLateralAxis`/`kVerticalAxis` in `gestures.cpp` encode
   an assumption about how the band is mounted on the wrist. If the
   physical orientation differs from "+X points away from the body, +Z
   points up," remap those constants (or insert a fixed axis-remapping
   rotation on the samples before they reach the recognizer) before doing
   any of the above -- otherwise you'll be tuning thresholds against the
   wrong axes.

There is no training pipeline and no persisted/EEPROM calibration store in
this firmware -- the calibration struct's values are compiled-in
constants. That's a deliberate simplicity choice for a first cut; if
per-operator calibration turns out to be needed, the natural extension is
loading `GestureCalibration` from NVS/Preferences at boot.

## Verifying E-stop latency with a logic analyser

Target: ESTOP packet on the air within 300 ms of the physical button
press. What "on the air" can mean two different things depending on what
you can instrument -- pick based on your logic analyzer's capabilities:

**Cheapest: button-to-GPIO-toggle proxy.** Add one line, temporarily, at
the top of `serviceEstopHighestPriority()` in `main.cpp`:
```cpp
digitalWrite(SOME_SPARE_GPIO, HIGH);  // ... existing send logic ...
digitalWrite(SOME_SPARE_GPIO, LOW);
```
Wire a spare GPIO to your logic analyzer along with the E-stop button
line itself. Trigger capture on the button's falling edge, measure to the
spare GPIO's pulse. This measures firmware-side latency (ISR + loop
scheduling + packet build) but NOT actual BLE radio transmit time -- it
tells you whether the *software* budget is being met, which is the part
this codebase controls.

**More rigorous: sniff the actual BLE notify.** Use an nRF Sniffer for
Bluetooth LE (or similar BLE sniffer hardware) on a second logic-analyzer
channel/capture alongside the button GPIO, filtered to the band's
advertised MAC and the `6b9e0002-...` characteristic handle. Trigger on
the button's falling edge, measure to the sniffed NOTIFY packet
containing `type == 3` (ESTOP) in the air. This is the number that
actually matters end-to-end, and it additionally captures BLE connection
interval effects that the GPIO-proxy method above cannot see -- see the
caveat below.

**Known caveat that the 300 ms target does not fully control for:** BLE
notifications are still gated by the negotiated connection interval. If
the drone's central connects with a long connection interval (some stacks
default to 30-50 ms or higher), the actual over-the-air delivery adds up
to one interval's worth of latency on top of firmware latency, and it is
NOT visible in the GPIO-proxy measurement above. When you do the
connection-parameter negotiation on the drone side, request a short
interval (7.5-15 ms) specifically because of this budget; this firmware
does not currently call `BLEDevice::updatePeerMTU`/connection-parameter
APIs itself to request that from its side, which is worth adding once the
drone-side stack is in place. Also note that this firmware issues the
BLE `notify()` call unconditionally on every loop iteration; it does not
implement its own queue/backpressure, so if a `notify()` targeting a
disconnected/unready link would take multiple milliseconds internally
that would show up in this measurement too -- if you see the pulse but no
sniffed radio packet, this is the first place to look.

## What is stubbed and untested

Read this list before trusting any part of this firmware near a person or
a flying drone.

- **UWB (`src/uwb.h`, `src/uwb.cpp`) is a stub, not a driver.**
  `UwbResponder::begin()` configures GPIO direction/idle state and arms
  the ESP32-side IRQ plumbing, but never talks to the DW3220 over SPI --
  it deliberately returns `false` and leaves `state_ ==
  RangingState::kNotInitialized` forever, so `uwb_ok` in the heartbeat
  will read `0` until real driver code lands. `uwb.cpp` has a long
  comment block naming the exact Qorvo DW3000-family driver calls
  (`dwt_initialise`, `dwt_configure`, `dwt_setrxantennadelay` /
  `dwt_settxantennadelay`, `dwt_setcallbacks`, `dwt_rxenable`,
  `dwt_setdelayedtrxtime` + `dwt_starttx(DWT_START_TX_DELAYED)`,
  `dwt_readrxtimestamp`/`dwt_readtxtimestamp`, `dwt_forcetrxoff`) that a
  real DS-TWR responder implementation needs, in the order it needs them,
  without guessing at struct layouts -- that work requires vendoring in
  Qorvo's actual DW3000 API package and is intentionally out of scope
  here.
- **Nothing in this firmware has been compiled.** PlatformIO was not
  available in the authoring environment. Expect at least minor
  fixes on first build (library version resolution, LEDC API version
  mismatch noted above, possibly BLE header include-path differences
  across arduino-esp32 core versions).
- **Gesture thresholds are guesses, not measurements.** Every constant in
  `GestureCalibration` (`src/gestures.h`) was chosen by inspection/
  estimation, not from recorded operator data. See "Calibrating the
  gesture thresholds" above -- do not deploy against a real operator
  without running that procedure first.
- **The forward/lateral/vertical axis mapping in `gestures.cpp` assumes a
  specific physical mounting orientation** (+X away from the body, +Z
  up). This has not been validated against an actual band on an actual
  wrist. If the mounting differs, gestures will be misclassified.
- **The E-stop <300ms target is a design target, not a measured result.**
  See "Verifying E-stop latency" above -- in particular, this firmware
  does not request a short BLE connection interval itself, so real
  end-to-end latency depends on how the drone side negotiates the
  connection.
- **No BLE bonding/security is configured.** The service is open
  (no pairing/encryption). Given this is a safety-relevant link, adding
  BLE security (at minimum "just works" pairing, ideally passkey/OOB) is
  worth prioritizing before field use, so an unrelated BLE central can't
  spoof haptic writes or, more importantly, so an attacker can't spoof
  gesture/heartbeat packets impersonating the band. Note this does NOT
  weaken the E-stop's own safety property -- the E-stop is a physical
  button read directly by this firmware, not something that arrives over
  BLE -- but it does mean gesture/heartbeat data (and haptic commands
  received) aren't authenticated today.
- **Battery divider ratio (`kBatteryDividerRatio` in `main.cpp`) is a
  placeholder (2.0, i.e. a 1:1 divider).** Set it to match the actual
  resistors on the board; otherwise `battery_mv` in the heartbeat is
  wrong by a constant, known-wrong factor.
- **IMU/UWB report-stream fusion is approximate.** The recognizer pairs
  each linear-acceleration sample with the most recently received game
  rotation vector; because the BNO085 reports the two independently, this
  pairing can be off by up to one report interval (~20 ms at 50 Hz). Not
  expected to matter for this recognizer's feature set (it uses
  integrated rotation over a whole segment, not per-sample fusion), but
  it's not been validated against real sensor data either.
- **No persisted calibration.** `GestureCalibration` values are compiled
  constants; there's no NVS/Preferences load path, so retuning requires a
  reflash. See "Calibrating the gesture thresholds" for the intended
  extension point if that becomes a problem.
- **protocol.h/.cpp is believed to be a faithful, standalone
  implementation of the spec'd wire format** (magic/version/type/seq/
  payload/CRC-8 poly 0x07 init 0x00 over bytes 0..N-2) and was written
  specifically so it has no Arduino dependency and can be reimplemented
  in Python and cross-checked byte-for-byte -- but that cross-check has
  not actually been done. Do that before trusting interoperability with
  the drone-side receiver.
