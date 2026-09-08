// gestures.h -- deterministic, hand-tuned gesture recognizer.
//
// Design rationale (see README for the full writeup): published gesture-UI
// studies commonly use a 4 s analysis window, which gives good classifier
// accuracy but a worst-case 4 s delay between the operator's intent and the
// drone reacting -- not acceptable when the "drone" is a multi-kg object
// flying near a person. We trade classifier sophistication for latency:
// a 0.8 s window (40 samples @ 50 Hz) with a lightweight rule-based
// classifier that runs in microseconds, so the dominant term in the
// response-time budget is the gesture's own physical duration, not
// inference time.
//
// This is intentionally NOT a neural network. It is an energy-based
// segmenter feeding a small hand-tuned decision tree over a handful of
// scalar features. That makes it auditable and tunable in the field
// without a training pipeline.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gestures {

// --- Gesture IDs -- MUST match the drone-side receiver exactly -----------
enum GestureId : uint8_t {
  kGestureNone = 0,
  kGestureFollow = 1,  // forward push of the forearm
  kGestureStop = 2,    // sharp downward chop
  kGestureOrbit = 3,   // horizontal circular wrist rotation
  kGestureLand = 4,    // slow downward sweep, held
  kGestureCome = 5,    // beckon: two quick pulls toward the body
};

// ===========================================================================
// CALIBRATION -- every magic number the recognizer uses lives HERE.
// These were chosen by inspection/estimation, not from operator data, and
// WILL need retuning against real operators and a real arm-mounted BNO085.
// See README.md "Calibrating the gesture thresholds" before trusting these.
// ===========================================================================
struct GestureCalibration {
  // --- Segmentation (energy-based start/end detection) ---
  // Linear-acceleration magnitude (m/s^2, gravity already removed by the
  // BNO085 SH2_LINEAR_ACCELERATION report) above which a gesture is judged
  // to have started.
  float startThresholdMps2 = 6.0f;

  // Magnitude below which a sample counts toward "gesture has ended".
  // Intentionally lower than startThresholdMps2 (hysteresis) so a signal
  // dithering right at the boundary doesn't chatter start/end.
  float endThresholdMps2 = 3.0f;

  // Consecutive low-energy samples required to close out a gesture.
  // At 50 Hz, 5 samples = 100 ms of quiescence.
  uint8_t endHoldSamples = 5;

  // Segments shorter than this are almost certainly noise/jostle, not an
  // intentional gesture -- discarded before classification even runs.
  uint8_t minGestureSamples = 6;   // 120 ms

  // Ring buffer / analysis window cap: 0.8 s @ 50 Hz. A segment that is
  // still "in gesture" at this many samples is force-classified rather
  // than grown further, which bounds worst-case recognizer latency.
  uint8_t maxGestureSamples = 40;  // 800 ms

  // Per-axis acceleration zero-crossing detector: a crossing is only
  // counted if the signal has moved at least this far past zero on both
  // sides, so sensor noise near zero doesn't inflate the crossing count.
  float zeroCrossingDeadbandMps2 = 0.5f;

  // Minimum confidence (0..255) required before a classification result
  // is actually emitted over BLE.
  uint8_t confidenceThreshold = 140;

  // Per-gesture-ID cooldown: once gesture X fires, gesture X is
  // suppressed for this many ms so a single continued motion can't
  // re-trigger the same command repeatedly. Different gesture IDs are NOT
  // cross-suppressed (a STOP should never be blocked by a recent FOLLOW).
  uint16_t perGestureCooldownMs = 600;

  // --- Per-gesture feature thresholds ---

  // ORBIT: integrated |yaw rate| over the segment, in radians, above
  // which we consider the wrist to have actually circled rather than
  // just wobbled. ~2.6 rad ~= 150 degrees of accumulated rotation.
  float orbitMinIntegratedYawRad = 2.6f;
  // ORBIT additionally requires a minimum number of horizontal-plane
  // zero crossings (periodicity signature of a circular motion).
  uint8_t orbitMinZeroCrossings = 3;

  // STOP: a "sharp chop" is short and forceful -- upper bound on
  // duration and lower bound on peak magnitude distinguish it from LAND.
  uint8_t stopMaxDurationSamples = 16;      // <= 320 ms
  float stopMinPeakMps2 = 9.0f;

  // LAND: "slow downward sweep, held" -- long duration, comparatively
  // gentle peak acceleration, dominant axis vertical/negative.
  uint8_t landMinDurationSamples = 24;      // >= 480 ms
  float landMaxPeakMps2 = 8.0f;

  // FOLLOW: forward push -- single decisive excursion on the forward
  // axis, few zero crossings (i.e. not oscillatory).
  uint8_t followMaxZeroCrossings = 1;
  float followMinPeakMps2 = 6.5f;

  // COME: beckon = two quick inward pulls -- look for >= this many
  // zero crossings on the forward axis within a short duration.
  uint8_t comeMinZeroCrossings = 3;
  uint8_t comeMaxDurationSamples = 32;      // <= 640 ms
};

// --- Sample type ------------------------------------------------------
// Deliberately decoupled from Adafruit_BNO08x's sh2_* types: main.cpp is
// the only file that talks to the IMU library, and converts each report
// into this plain struct. Keeps gestures.cpp unit-testable without
// pulling in the sensor driver.
struct ImuSample {
  float ax, ay, az;      // linear acceleration, m/s^2, gravity removed
  float qw, qx, qy, qz;  // game rotation vector (unit quaternion)
  uint32_t t_ms;         // millis() timestamp this sample was captured
};

// Result of a completed classification, valid only when `valid` is true.
struct GestureResult {
  bool valid = false;
  GestureId id = kGestureNone;
  uint8_t confidence = 0;  // 0..255
};

// Ring buffer capacity matches maxGestureSamples's default; sized as a
// compile-time constant (not tied to the mutable calibration value) so
// storage is static and allocation-free.
constexpr size_t kRingCapacity = 40;

class GestureRecognizer {
 public:
  explicit GestureRecognizer(const GestureCalibration& calibration = GestureCalibration());

  // Feeds one new IMU sample (call at the IMU's native ~50 Hz rate).
  // Returns a GestureResult; `valid` is true only on the sample where a
  // segment was just classified AND passed the confidence threshold AND
  // is not within its own per-gesture cooldown. Every other call returns
  // `valid == false`. No dynamic allocation; safe to call from loop().
  GestureResult update(const ImuSample& sample);

  // Mutable access for runtime/serial-console calibration tuning.
  GestureCalibration& calibration() { return cal_; }
  const GestureCalibration& calibration() const { return cal_; }

 private:
  enum class State : uint8_t { kIdle, kInGesture };

  GestureCalibration cal_;
  State state_ = State::kIdle;

  // Ring buffer of recent samples. `head_` is the index the NEXT sample
  // will be written to; the buffer always holds the most recent
  // kRingCapacity samples (older ones are simply overwritten).
  ImuSample ring_[kRingCapacity];
  size_t head_ = 0;
  size_t count_ = 0;  // number of valid samples currently in ring_ (<= kRingCapacity)

  // Segment-in-progress bookkeeping (reset on kIdle -> kInGesture).
  size_t segmentStartIndex_ = 0;  // ring_ index of first sample in the segment
  size_t segmentLength_ = 0;      // number of samples accumulated so far
  uint8_t lowEnergyRun_ = 0;      // consecutive low-energy samples seen

  // Last-fired timestamp per gesture ID, for cooldown enforcement.
  // Index 0 unused (kGestureNone).
  uint32_t lastFiredMs_[6] = {0, 0, 0, 0, 0, 0};

  void resetSegment();
  GestureResult classifySegment(uint32_t nowMs);
  static float accelMagnitude(const ImuSample& s);
};

}  // namespace gestures
