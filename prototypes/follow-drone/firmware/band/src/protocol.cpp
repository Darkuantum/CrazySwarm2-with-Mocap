#include "protocol.h"

namespace protocol {

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;  // init
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      // Poly 0x07 (x^8 + x^2 + x^1 + x^0), MSB-first, no reflection.
      if (crc & 0x80) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

namespace {

// Shared header-writing logic for every encoder. Returns the offset just
// past the header (== kHeaderLen) so callers append payload bytes there.
size_t writeHeader(uint8_t* out, PacketType type, uint16_t seq) {
  out[0] = kMagic;
  out[1] = kVersion;
  out[2] = static_cast<uint8_t>(type);
  out[3] = static_cast<uint8_t>(seq & 0xFF);         // seq low byte
  out[4] = static_cast<uint8_t>((seq >> 8) & 0xFF);  // seq high byte
  return kHeaderLen;
}

// Appends the CRC over out[0..pos-1] at out[pos], returns pos+1 (total
// frame length), or 0 if there isn't room for the CRC byte.
size_t finalize(uint8_t* out, size_t out_capacity, size_t pos) {
  if (pos + kCrcLen > out_capacity) return 0;
  out[pos] = crc8(out, pos);
  return pos + kCrcLen;
}

}  // namespace

size_t encodeHeartbeat(uint8_t* out, size_t out_capacity, uint16_t seq,
                        const HeartbeatPayload& payload) {
  if (out_capacity < kHeaderLen + kHeartbeatPayloadLen + kCrcLen) return 0;
  size_t pos = writeHeader(out, PacketType::kHeartbeat, seq);
  out[pos++] = static_cast<uint8_t>(payload.battery_mv & 0xFF);
  out[pos++] = static_cast<uint8_t>((payload.battery_mv >> 8) & 0xFF);
  out[pos++] = payload.imu_ok;
  out[pos++] = payload.uwb_ok;
  out[pos++] = payload.estop_latched;
  return finalize(out, out_capacity, pos);
}

size_t encodeGesture(uint8_t* out, size_t out_capacity, uint16_t seq,
                      const GesturePayload& payload) {
  if (out_capacity < kHeaderLen + kGesturePayloadLen + kCrcLen) return 0;
  size_t pos = writeHeader(out, PacketType::kGesture, seq);
  out[pos++] = payload.gesture_id;
  out[pos++] = payload.confidence;
  return finalize(out, out_capacity, pos);
}

size_t encodeEstop(uint8_t* out, size_t out_capacity, uint16_t seq,
                    const EstopPayload& payload) {
  if (out_capacity < kHeaderLen + kEstopPayloadLen + kCrcLen) return 0;
  size_t pos = writeHeader(out, PacketType::kEstop, seq);
  out[pos++] = payload.pressed;
  out[pos++] = static_cast<uint8_t>(payload.press_count & 0xFF);
  out[pos++] = static_cast<uint8_t>((payload.press_count >> 8) & 0xFF);
  return finalize(out, out_capacity, pos);
}

bool decode(const uint8_t* data, size_t len, DecodedHeader* out) {
  if (data == nullptr || out == nullptr) return false;
  if (len < kHeaderLen + kCrcLen) return false;
  if (data[0] != kMagic) return false;
  if (data[1] != kVersion) return false;

  const uint8_t rawType = data[2];
  if (rawType != static_cast<uint8_t>(PacketType::kHeartbeat) &&
      rawType != static_cast<uint8_t>(PacketType::kGesture) &&
      rawType != static_cast<uint8_t>(PacketType::kEstop) &&
      rawType != static_cast<uint8_t>(PacketType::kAck)) {
    return false;
  }

  // CRC covers everything except the trailing CRC byte itself, i.e.
  // bytes [0, len-1).
  const uint8_t expectedCrc = data[len - 1];
  const uint8_t actualCrc = crc8(data, len - kCrcLen);
  if (expectedCrc != actualCrc) return false;

  out->version = data[1];
  out->type = static_cast<PacketType>(rawType);
  out->seq = static_cast<uint16_t>(data[3]) | (static_cast<uint16_t>(data[4]) << 8);
  out->payload = data + kHeaderLen;
  out->payload_len = len - kHeaderLen - kCrcLen;
  return true;
}

}  // namespace protocol
