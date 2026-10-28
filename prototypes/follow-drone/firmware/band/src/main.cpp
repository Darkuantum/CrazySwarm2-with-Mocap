// main.cpp -- wearable gesture/ranging band firmware.
//
// Responsibilities tied together here:
//   - Hardware E-STOP: interrupt-driven, latching, transmitted ahead of
//     everything else. This is the ONE thing in this file that must never
//     be made to wait on BLE connection state, gesture processing, or the
//     normal loop cadence. See serviceEstopHighestPriority() below and the
//     README's "verifying E-stop latency" section.
//   - BNO085 IMU sampling -> gesture recognizer -> BLE gesture notify.
//   - DW3220 UWB responder polling (stubbed; see uwb.cpp).
//   - BLE peripheral: NOTIFY characteristic for our outgoing packets,
//     WRITE characteristic for the drone's haptic-feedback commands.
//   - 5 Hz heartbeat scheduling.
//
// Every packet placed on the air uses the framing in protocol.h.
#include <Arduino.h>
#include <Wire.h>
#include <string>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <Adafruit_BNO08x.h>

#include "gestures.h"
#include "haptics.h"
#include "protocol.h"
#include "uwb.h"

// ---------------------------------------------------------------------
// Pin assignments (see README for the full pinout table).
// ---------------------------------------------------------------------
namespace pins {
constexpr int kImuSda = 21;
constexpr int kImuScl = 22;
constexpr int kImuReset = -1;  // BNO085 reset tied to board reset; no GPIO used

constexpr int kUwbSck = 18;
constexpr int kUwbMiso = 19;
constexpr int kUwbMosi = 23;
constexpr int kUwbCs = 5;
constexpr int kUwbIrq = 34;
constexpr int kUwbRst = 27;

constexpr int kEstopButton = 33;
constexpr int kVibeMotor = 25;
constexpr int kBatterySense = 35;
}  // namespace pins

// ---------------------------------------------------------------------
// BLE UUIDs / roles (fixed contract with the drone-side receiver).
// ---------------------------------------------------------------------
namespace ble_uuid {
constexpr char kService[] = "6b9e0001-4f3a-4b1e-9c2d-1a7f5e8c3d90";
constexpr char kCmdNotify[] = "6b9e0002-4f3a-4b1e-9c2d-1a7f5e8c3d90";
constexpr char kHapticWrite[] = "6b9e0003-4f3a-4b1e-9c2d-1a7f5e8c3d90";
}  // namespace ble_uuid

// ---------------------------------------------------------------------
// Timing constants.
// ---------------------------------------------------------------------
constexpr uint32_t kHeartbeatIntervalMs = 200;  // 5 Hz, per spec
constexpr uint32_t kImuSampleIntervalUs = 20000;  // 50 Hz report rate from the BNO085
constexpr uint32_t kImuStaleTimeoutMs = 500;  // no fresh sample this long -> imu_ok = false

// Debounce window for the E-stop button. Mechanical button bounce is
// typically <10ms; 30ms gives margin without meaningfully eating into the
// 300ms transmit budget.
constexpr uint32_t kEstopDebounceMs = 30;
// Hold time required to CLEAR a latched E-stop. Deliberately long (2s) so
// clearing can never happen by accident -- asserting the stop must always
// be much easier than releasing it.
constexpr uint32_t kEstopClearHoldMs = 2000;

// Battery divider: adjust to the actual resistor values populated on the
// board. Assumes a 2:1 divider (equal legs) feeding the ADC, i.e.
// V_batt = V_adc * 2. ESP32 ADC configured for its widest attenuation
// (~0-3.3V full scale) below.
constexpr float kBatteryDividerRatio = 2.0f;
constexpr float kAdcRefMv = 3300.0f;
constexpr float kAdcMaxCounts = 4095.0f;

// ---------------------------------------------------------------------
// Globals. Kept as file-scope statics (not function-local) because they
// are shared between the ISR, BLE callback context, and loop() -- making
// that sharing explicit is more honest than hiding it behind "clever"
// scoping.
// ---------------------------------------------------------------------

// -- E-stop state (ISR-shared; guarded by g_estopMux) --
static portMUX_TYPE g_estopMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_estopLatched = false;
static volatile bool g_estopPressEventPending = false;  // loop must send ESTOP now
static volatile uint16_t g_estopPressCount = 0;
static volatile uint32_t g_estopLastIsrMs = 0;  // ISR-side debounce clock

// -- BLE --
static BLEServer* g_bleServer = nullptr;
static BLECharacteristic* g_cmdCharacteristic = nullptr;
static BLECharacteristic* g_hapticCharacteristic = nullptr;
static volatile bool g_bleConnected = false;

// -- Sensors / subsystems --
static Adafruit_BNO08x g_bno08x;
static gestures::GestureRecognizer g_recognizer;
static uwb::UwbResponder g_uwbResponder(uwb::Pins{
    pins::kUwbSck, pins::kUwbMiso, pins::kUwbMosi, pins::kUwbCs, pins::kUwbIrq,
    pins::kUwbRst});
static haptics::HapticDriver g_hapticDriver(haptics::Pins{pins::kVibeMotor});

// -- IMU freshness / fusion of the two independent BNO08x report streams --
static bool g_imuInitOk = false;
static uint32_t g_lastImuSampleMs = 0;
static float g_latestQw = 1.0f, g_latestQx = 0.0f, g_latestQy = 0.0f, g_latestQz = 0.0f;
static bool g_haveQuaternion = false;

// -- Packet sequence number: one counter shared by every packet type, so
// the drone side can detect gaps/reordering across the whole stream. --
static uint16_t g_seq = 0;

// -- Scratch TX buffer. Reused for every send: loop() is single-threaded
// and packets are built-then-sent-immediately, so there is no aliasing
// hazard in sharing one buffer. --
static uint8_t g_txBuf[protocol::kMaxPacketLen];

// ---------------------------------------------------------------------
// BLE send helpers.
// ---------------------------------------------------------------------

static void bleNotify(const uint8_t* data, size_t len) {
  if (!g_bleConnected || g_cmdCharacteristic == nullptr) return;
  // setValue() copies into the characteristic's internal buffer, so it's
  // fine that g_txBuf gets reused on the next call.
  g_cmdCharacteristic->setValue(const_cast<uint8_t*>(data), len);
  g_cmdCharacteristic->notify();
}

static void sendEstopPacket(uint8_t pressed, uint16_t pressCount) {
  protocol::EstopPayload payload{pressed, pressCount};
  size_t len = protocol::encodeEstop(g_txBuf, sizeof(g_txBuf), g_seq++, payload);
  if (len > 0) bleNotify(g_txBuf, len);
}

static void sendGesturePacket(uint8_t gestureId, uint8_t confidence) {
  protocol::GesturePayload payload{gestureId, confidence};
  size_t len = protocol::encodeGesture(g_txBuf, sizeof(g_txBuf), g_seq++, payload);
  if (len > 0) bleNotify(g_txBuf, len);
}

static void sendHeartbeatPacket(const protocol::HeartbeatPayload& payload) {
  size_t len = protocol::encodeHeartbeat(g_txBuf, sizeof(g_txBuf), g_seq++, payload);
  if (len > 0) bleNotify(g_txBuf, len);
}

// ---------------------------------------------------------------------
// E-stop: ISR + latch + highest-priority transmit.
//
// SAFETY CONTRACT (do not weaken this without re-reading the spec):
//   - The ISR only debounces and latches; it never touches BLE/SPI.
//   - serviceEstopHighestPriority() is the FIRST thing loop() does, every
//     iteration, unconditionally -- it does not check gesture state, BLE
//     connection state, or any other subsystem before running.
//   - Once latched, every heartbeat tick re-sends an ESTOP packet instead
//     of a HEARTBEAT until an explicit 2s-hold clear.
// ---------------------------------------------------------------------

// IRAM_ATTR: this may fire while flash is busy elsewhere; keep it tiny
// and branch-predictable. millis() is safe to call from an ISR on the
// Arduino-ESP32 core (it reads a hardware/RTOS tick source, not flash).
static void IRAM_ATTR onEstopFallingIsr() {
  portENTER_CRITICAL_ISR(&g_estopMux);
  const uint32_t now = millis();
  if (now - g_estopLastIsrMs >= kEstopDebounceMs) {
    g_estopLastIsrMs = now;
    g_estopLatched = true;
    g_estopPressCount++;
    g_estopPressEventPending = true;
  }
  portEXIT_CRITICAL_ISR(&g_estopMux);
}

// Called first in loop(), every iteration. Building+sending a BLE notify
// is the earliest point after the ISR where we're legally allowed to
// touch the radio, so this is the critical link in the <300ms budget --
// see README for how to measure the actual end-to-end number.
static void serviceEstopHighestPriority() {
  bool sendNow = false;
  uint16_t countSnapshot = 0;

  portENTER_CRITICAL(&g_estopMux);
  if (g_estopPressEventPending) {
    g_estopPressEventPending = false;
    sendNow = true;
    countSnapshot = g_estopPressCount;
  }
  portEXIT_CRITICAL(&g_estopMux);

  if (sendNow) {
    sendEstopPacket(/*pressed=*/1, countSnapshot);
  }
}

// Long-press-to-clear. This is deliberately NOT interrupt-driven: the
// button only generates a FALLING edge on press, so detecting a 2s HOLD
// requires level polling. That's fine -- clearing a latch is not on the
// latency-critical path, only asserting it is.
static void serviceEstopClearHold() {
  static bool heldLevel = false;
  static uint32_t holdStartMs = 0;

  const bool pinLow = (digitalRead(pins::kEstopButton) == LOW);  // active-low
  if (pinLow && !heldLevel) {
    heldLevel = true;
    holdStartMs = millis();
  } else if (!pinLow) {
    heldLevel = false;
  }

  bool latchedNow;
  portENTER_CRITICAL(&g_estopMux);
  latchedNow = g_estopLatched;
  portEXIT_CRITICAL(&g_estopMux);

  if (latchedNow && heldLevel && (millis() - holdStartMs >= kEstopClearHoldMs)) {
    uint16_t countSnapshot;
    portENTER_CRITICAL(&g_estopMux);
    g_estopLatched = false;
    countSnapshot = g_estopPressCount;
    portEXIT_CRITICAL(&g_estopMux);

    heldLevel = false;  // require the button to be released and re-pressed
                         // before another clear can be armed
    sendEstopPacket(/*pressed=*/0, countSnapshot);
  }
}

// ---------------------------------------------------------------------
// Heartbeat (5 Hz). When latched, this is what makes the ESTOP condition
// durable on the wire -- a single dropped BLE notify does not lose the
// stop, the very next tick (200ms later) resends it.
// ---------------------------------------------------------------------

static uint16_t readBatteryMilliVolts() {
  const int raw = analogRead(pins::kBatterySense);
  const float adcMv = (raw / kAdcMaxCounts) * kAdcRefMv;
  return static_cast<uint16_t>(adcMv * kBatteryDividerRatio);
}

static void serviceHeartbeatTick() {
  static uint32_t lastTickMs = 0;
  const uint32_t now = millis();
  if (now - lastTickMs < kHeartbeatIntervalMs) return;
  lastTickMs = now;

  bool latchedNow;
  uint16_t countSnapshot;
  portENTER_CRITICAL(&g_estopMux);
  latchedNow = g_estopLatched;
  countSnapshot = g_estopPressCount;
  portEXIT_CRITICAL(&g_estopMux);

  if (latchedNow) {
    // Requirement: keep re-sending ESTOP in every heartbeat until cleared.
    sendEstopPacket(/*pressed=*/1, countSnapshot);
    return;
  }

  const bool imuFresh = (now - g_lastImuSampleMs) < kImuStaleTimeoutMs;
  protocol::HeartbeatPayload payload;
  payload.battery_mv = readBatteryMilliVolts();
  payload.imu_ok = (g_imuInitOk && imuFresh) ? 1 : 0;
  payload.uwb_ok = g_uwbResponder.isHealthy() ? 1 : 0;
  payload.estop_latched = 0;
  sendHeartbeatPacket(payload);
}

// ---------------------------------------------------------------------
// IMU: pump BNO08x events, fuse the two report streams into gesture
// samples, run the recognizer, transmit on a positive classification.
// ---------------------------------------------------------------------

static void configureImuReports() {
  // Both reports at the same 50Hz cadence the gesture recognizer expects.
  // If either enableReport() call fails, g_imuInitOk stays false and the
  // heartbeat will honestly report imu_ok = 0.
  g_imuInitOk = g_bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, kImuSampleIntervalUs) &&
                g_bno08x.enableReport(SH2_LINEAR_ACCELERATION, kImuSampleIntervalUs);
}

static void handleGestureResult(const gestures::GestureResult& result) {
  if (!result.valid) return;
  sendGesturePacket(static_cast<uint8_t>(result.id), result.confidence);
}

static void serviceImu() {
  sh2_SensorValue_t event;
  // Drain all pending events each loop pass rather than at most one, so a
  // burst of buffered reports (e.g. after a slow loop iteration) doesn't
  // starve the recognizer of samples or introduce artificial latency.
  while (g_bno08x.getSensorEvent(&event)) {
    switch (event.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        g_latestQw = event.un.gameRotationVector.real;
        g_latestQx = event.un.gameRotationVector.i;
        g_latestQy = event.un.gameRotationVector.j;
        g_latestQz = event.un.gameRotationVector.k;
        g_haveQuaternion = true;
        break;

      case SH2_LINEAR_ACCELERATION: {
        // Linear-acceleration events are our 50Hz sample clock; pair each
        // one with the most recently received quaternion. The two report
        // streams are independently timestamped by the BNO085, so this
        // pairing can be off by up to one report interval (~20ms) -- fine
        // for this recognizer's feature set, but worth knowing if this
        // code is ever reused somewhere timing-sensitive.
        gestures::ImuSample sample;
        sample.ax = event.un.linearAcceleration.x;
        sample.ay = event.un.linearAcceleration.y;
        sample.az = event.un.linearAcceleration.z;
        sample.qw = g_latestQw;
        sample.qx = g_latestQx;
        sample.qy = g_latestQy;
        sample.qz = g_latestQz;
        sample.t_ms = millis();

        g_lastImuSampleMs = sample.t_ms;

        if (g_haveQuaternion) {
          gestures::GestureResult result = g_recognizer.update(sample);
          handleGestureResult(result);
        }
        break;
      }

      default:
        break;
    }
  }
}

// ---------------------------------------------------------------------
// BLE server/characteristic callbacks.
// ---------------------------------------------------------------------

class BandServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    g_bleConnected = true;
  }
  void onDisconnect(BLEServer* server) override {
    g_bleConnected = false;
    // Resume advertising so the drone can reconnect after a link drop.
    // Note this does NOT affect E-stop behavior: the latch and its
    // resend-on-heartbeat logic run regardless of connection state, so
    // the very next successful connection immediately sees a latched
    // ESTOP if one occurred while disconnected.
    server->getAdvertising()->start();
  }
};

class HapticWriteCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    if (value.empty()) return;
    // Single raw byte, per spec: 1/2/3 select a feedback pattern. Not
    // wrapped in the protocol.h framing -- that framing is for our
    // outgoing telemetry stream, the haptic command is intentionally the
    // simplest possible write.
    g_hapticDriver.trigger(static_cast<haptics::PatternId>(
        static_cast<uint8_t>(value[0])));
  }
};

static void setupBle() {
  BLEDevice::init("FollowDroneBand");
  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new BandServerCallbacks());

  BLEService* service = g_bleServer->createService(ble_uuid::kService);

  g_cmdCharacteristic = service->createCharacteristic(
      ble_uuid::kCmdNotify, BLECharacteristic::PROPERTY_NOTIFY);
  g_cmdCharacteristic->addDescriptor(new BLE2902());

  g_hapticCharacteristic = service->createCharacteristic(
      ble_uuid::kHapticWrite, BLECharacteristic::PROPERTY_WRITE);
  g_hapticCharacteristic->setCallbacks(new HapticWriteCallbacks());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(ble_uuid::kService);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ---------------------------------------------------------------------
// setup() / loop()
// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // E-stop button first, before anything else that could conceivably
  // delay startup (BLE/IMU/UWB bring-up) -- if the button is already
  // held at power-on, we still want the interrupt armed as early as
  // possible.
  pinMode(pins::kEstopButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::kEstopButton), onEstopFallingIsr, FALLING);

  // ESP32 ADC: widest attenuation so the ~0-3.3V divider output is inside
  // the usable range (default attenuation clips well below 3.3V).
  analogSetPinAttenuation(pins::kBatterySense, ADC_11db);

  g_hapticDriver.begin();

  Wire.begin(pins::kImuSda, pins::kImuScl);
  g_imuInitOk = g_bno08x.begin_I2C();
  if (g_imuInitOk) {
    configureImuReports();
  }

  g_uwbResponder.begin();  // stubbed today; see uwb.cpp. Failure is not fatal
                           // -- the band still functions as a gesture/E-stop
                           // controller without ranging, and uwb_ok simply
                           // reads false in the heartbeat.

  setupBle();
}

void loop() {
  // ORDER MATTERS. E-stop handling is unconditionally first: it must
  // never be delayed behind gesture processing, UWB polling, or BLE
  // connection bookkeeping.
  serviceEstopHighestPriority();
  serviceEstopClearHold();

  serviceImu();
  g_uwbResponder.poll();
  g_hapticDriver.poll();
  serviceHeartbeatTick();

  // Deliberately no delay() here: every subsystem above is either
  // event-driven (IMU/UWB poll a hardware flag and return immediately
  // when idle) or self-timed via millis() comparisons, so looping as
  // fast as possible costs CPU but never blocks anything else. If power
  // consumption becomes a concern, add a short `delay(1)` here -- but
  // re-measure the E-stop latency budget in the README before doing so.
}
