#include "gestures.h"

#include <math.h>

namespace gestures {

namespace {

// Body-axis assignment for the IMU as worn on the forearm. This is a
// PHYSICAL MOUNTING ASSUMPTION, not a general truth -- if the band ends up
// mounted rotated relative to this, either remap axes here or add a fixed
// rotation to the samples before they reach the recognizer. See README.
constexpr int kForwardAxis = 0;   // +X: away from the body (push/beckon axis)
constexpr int kLateralAxis = 1;   // +Y: sideways
constexpr int kVerticalAxis = 2;  // +Z: up; a chop/sweep is negative Z

constexpr float kPi = 3.14159265358979323846f;

float yawFromQuaternion(float qw, float qx, float qy, float qz) {
  // Standard quaternion-to-yaw (rotation about the vertical/Z body axis),
  // range (-pi, pi]. We only need yaw (for ORBIT); pitch/roll are unused
  // by the current gesture set.
  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  return atan2f(siny_cosp, cosy_cosp);
}

// Maps how far `value` clears `threshold`, scaled by `span`, into 0..255.
// value <= threshold -> 0; value >= threshold + span -> 255; linear between.
uint8_t marginScore(float value, float threshold, float span) {
  if (span <= 0.0f) return (value >= threshold) ? 255 : 0;
  float t = (value - threshold) / span;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return static_cast<uint8_t>(t * 255.0f + 0.5f);
}

uint8_t averageScore(uint8_t a, uint8_t b) {
  return static_cast<uint8_t>((static_cast<uint16_t>(a) + b) / 2);
}

}  // namespace

GestureRecognizer::GestureRecognizer(const GestureCalibration& calibration)
    : cal_(calibration) {}

float GestureRecognizer::accelMagnitude(const ImuSample& s) {
  return sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
}

void GestureRecognizer::resetSegment() {
  segmentLength_ = 0;
  lowEnergyRun_ = 0;
}

GestureResult GestureRecognizer::update(const ImuSample& sample) {
  GestureResult none;

  // Always record the sample, regardless of segmentation state, so the
  // ring buffer keeps rolling context (useful for future features/logging
  // even though the current classifier only reads within an active
  // segment).
  const size_t writtenIndex = head_;
  ring_[head_] = sample;
  head_ = (head_ + 1) % kRingCapacity;
  if (count_ < kRingCapacity) ++count_;

  const float mag = accelMagnitude(sample);

  if (state_ == State::kIdle) {
    if (mag > cal_.startThresholdMps2) {
      state_ = State::kInGesture;
      segmentStartIndex_ = writtenIndex;
      segmentLength_ = 1;
      lowEnergyRun_ = 0;
    }
    return none;
  }

  // state_ == kInGesture
  ++segmentLength_;
  if (segmentLength_ > kRingCapacity) {
    // Defensive clamp: only reachable if maxGestureSamples is misconfigured
    // above kRingCapacity, in which case the oldest samples have already
    // been overwritten by the ring buffer. Slide the window start forward
    // so segmentStartIndex_/segmentLength_ still describe valid data.
    segmentLength_ = kRingCapacity;
    segmentStartIndex_ = (segmentStartIndex_ + 1) % kRingCapacity;
  }

  lowEnergyRun_ = (mag < cal_.endThresholdMps2) ? (lowEnergyRun_ + 1) : 0;

  const bool naturalEnd = lowEnergyRun_ >= cal_.endHoldSamples;
  const bool forceEnd = segmentLength_ >= cal_.maxGestureSamples;

  if (!naturalEnd && !forceEnd) {
    return none;
  }

  // Trim the trailing quiescent tail (the samples that told us the
  // gesture ended) so duration/peak features describe the motion itself.
  if (naturalEnd && segmentLength_ > lowEnergyRun_) {
    segmentLength_ -= lowEnergyRun_;
  }

  GestureResult result = classifySegment(sample.t_ms);
  state_ = State::kIdle;
  resetSegment();
  return result;
}

GestureResult GestureRecognizer::classifySegment(uint32_t nowMs) {
  GestureResult result;  // valid=false by default

  if (segmentLength_ < cal_.minGestureSamples) {
    return result;  // too short to be an intentional gesture -- discard
  }

  // --- Feature extraction: single pass over the segment ------------------
  float maxAbs[3] = {0.0f, 0.0f, 0.0f};
  float peakSigned[3] = {0.0f, 0.0f, 0.0f};
  uint16_t crossings[3] = {0, 0, 0};
  float lastCrossingSign[3] = {0.0f, 0.0f, 0.0f};
  bool haveCrossingSign[3] = {false, false, false};
  float sumAbsYawDelta = 0.0f;
  float prevYaw = 0.0f;
  bool haveYaw = false;

  for (size_t i = 0; i < segmentLength_; ++i) {
    const size_t idx = (segmentStartIndex_ + i) % kRingCapacity;
    const ImuSample& s = ring_[idx];
    const float axisVal[3] = {s.ax, s.ay, s.az};

    for (int a = 0; a < 3; ++a) {
      const float v = axisVal[a];
      const float av = fabsf(v);
      if (av > maxAbs[a]) {
        maxAbs[a] = av;
        peakSigned[a] = v;
      }
      if (av >= cal_.zeroCrossingDeadbandMps2) {
        if (haveCrossingSign[a] && ((lastCrossingSign[a] > 0.0f) != (v > 0.0f))) {
          ++crossings[a];
        }
        lastCrossingSign[a] = v;
        haveCrossingSign[a] = true;
      }
    }

    const float yaw = yawFromQuaternion(s.qw, s.qx, s.qy, s.qz);
    if (haveYaw) {
      float delta = yaw - prevYaw;
      if (delta > kPi) delta -= 2.0f * kPi;
      if (delta < -kPi) delta += 2.0f * kPi;
      sumAbsYawDelta += fabsf(delta);
    }
    prevYaw = yaw;
    haveYaw = true;
  }

  int peakAxis = 0;
  if (maxAbs[1] > maxAbs[peakAxis]) peakAxis = 1;
  if (maxAbs[2] > maxAbs[peakAxis]) peakAxis = 2;
  const float peakVal = peakSigned[peakAxis];
  const float peakMag = maxAbs[peakAxis];

  const ImuSample& firstSample = ring_[segmentStartIndex_];
  const ImuSample& lastSample =
      ring_[(segmentStartIndex_ + segmentLength_ - 1) % kRingCapacity];
  const uint32_t durationMs = lastSample.t_ms - firstSample.t_ms;
  (void)durationMs;  // available for future features / logging

  // --- Per-gesture candidate scoring --------------------------------------
  // Each candidate is 0 unless its hard (binary) constraints hold, in
  // which case it's a soft score in 0..255 built from how far the
  // measured features clear their thresholds. The highest-scoring
  // candidate wins; a tie favors the lower numeric GestureId (see the
  // strict `>` comparison in the selection loop below). In practice ties
  // are rare since the hard constraints above are largely mutually
  // exclusive (e.g. STOP/LAND split on duration, FOLLOW/COME split on
  // zero-crossing count).

  uint8_t score[6] = {0, 0, 0, 0, 0, 0};  // indexed by GestureId

  // STOP: sharp downward chop -- vertical, negative, short, forceful,
  // not oscillatory.
  if (peakAxis == kVerticalAxis && peakVal < 0.0f &&
      segmentLength_ <= cal_.stopMaxDurationSamples &&
      peakMag >= cal_.stopMinPeakMps2 && crossings[kVerticalAxis] <= 1) {
    const uint8_t sMag = marginScore(peakMag, cal_.stopMinPeakMps2, 6.0f);
    const uint8_t sSharp = marginScore(
        static_cast<float>(cal_.stopMaxDurationSamples - segmentLength_), 0.0f,
        static_cast<float>(cal_.stopMaxDurationSamples));
    score[kGestureStop] = averageScore(sMag, sSharp);
  }

  // LAND: slow downward sweep, held -- vertical, negative, long, gentle.
  if (peakAxis == kVerticalAxis && peakVal < 0.0f &&
      segmentLength_ >= cal_.landMinDurationSamples &&
      peakMag <= cal_.landMaxPeakMps2) {
    const uint8_t sDur = marginScore(
        static_cast<float>(segmentLength_),
        static_cast<float>(cal_.landMinDurationSamples),
        static_cast<float>(cal_.maxGestureSamples - cal_.landMinDurationSamples));
    const uint8_t sGentle =
        marginScore(cal_.landMaxPeakMps2 - peakMag, 0.0f, cal_.landMaxPeakMps2);
    score[kGestureLand] = averageScore(sDur, sGentle);
  }

  // FOLLOW: single decisive forward push -- forward axis, positive
  // (away from body), not oscillatory.
  if (peakAxis == kForwardAxis && peakVal > 0.0f &&
      crossings[kForwardAxis] <= cal_.followMaxZeroCrossings &&
      peakMag >= cal_.followMinPeakMps2) {
    score[kGestureFollow] = marginScore(peakMag, cal_.followMinPeakMps2, 6.0f);
  }

  // COME: beckon -- two (or more) quick pulls on the forward axis, i.e.
  // several zero crossings, within a short-ish window. Direction of the
  // very first excursion isn't constrained since a beckon can start with
  // either the "reach" or the "pull".
  if (peakAxis == kForwardAxis &&
      crossings[kForwardAxis] >= cal_.comeMinZeroCrossings &&
      segmentLength_ <= cal_.comeMaxDurationSamples) {
    const uint8_t sCross = marginScore(
        static_cast<float>(crossings[kForwardAxis]),
        static_cast<float>(cal_.comeMinZeroCrossings), 3.0f);
    score[kGestureCome] = sCross;
  }

  // ORBIT: horizontal circular wrist rotation -- large integrated yaw,
  // periodic signature (crossings) on the horizontal axes.
  if (sumAbsYawDelta >= cal_.orbitMinIntegratedYawRad &&
      (crossings[kForwardAxis] + crossings[kLateralAxis]) >=
          cal_.orbitMinZeroCrossings) {
    const uint8_t sYaw =
        marginScore(sumAbsYawDelta, cal_.orbitMinIntegratedYawRad, 2.0f);
    const uint8_t sCross = marginScore(
        static_cast<float>(crossings[kForwardAxis] + crossings[kLateralAxis]),
        static_cast<float>(cal_.orbitMinZeroCrossings), 3.0f);
    score[kGestureOrbit] = averageScore(sYaw, sCross);
  }

  // --- Pick the winner -----------------------------------------------------
  GestureId bestId = kGestureNone;
  uint8_t bestScore = 0;
  for (int id = kGestureFollow; id <= kGestureCome; ++id) {
    if (score[id] > bestScore) {
      bestScore = score[id];
      bestId = static_cast<GestureId>(id);
    }
  }

  if (bestId == kGestureNone || bestScore < cal_.confidenceThreshold) {
    return result;  // no candidate cleared the bar
  }

  // Per-gesture cooldown: suppress a repeat of the SAME gesture ID while
  // it's within its cooldown window. We do not fall back to the runner-up
  // candidate -- if the operator is still mid-motion, letting a different
  // gesture ID fire instead could be more surprising than firing nothing.
  const uint32_t elapsed = nowMs - lastFiredMs_[bestId];
  if (elapsed < cal_.perGestureCooldownMs) {
    return result;
  }

  lastFiredMs_[bestId] = nowMs;
  result.valid = true;
  result.id = bestId;
  result.confidence = bestScore;
  return result;
}

}  // namespace gestures
