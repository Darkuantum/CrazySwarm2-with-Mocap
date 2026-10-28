// protocol.h -- wire format shared with the drone-side receiver.
//
// This header intentionally contains ONLY plain data layout and pure
// functions (no Arduino types, no BLE types) so that the exact same framing
// and CRC logic can be re-implemented in Python (e.g. with `struct` and a
// matching crc8()) and cross-checked byte-for-byte against this code in a
// unit test. Do not let BLE-specific concerns leak into this file.
//
// Frame layout (all multi-byte fields little-endian):
//   byte 0        : magic         = 0xFB
//   byte 1        : version       = 1
//   byte 2        : type          (see PacketType)
//   bytes 3-4     : sequence      uint16, little-endian
//   bytes 5..N-2  : payload       (type-specific, see *_Payload structs)
//   byte N-1      : crc8          CRC-8/SMBUS: poly 0x07, init 0x00,
//                                 computed over bytes 0..N-2 inclusive
//                                 (i.e. everything except the CRC byte itself)
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace protocol {

constexpr uint8_t kMagic = 0xFB;
constexpr uint8_t kVersion = 1;

// Header = magic, version, type, seq_lo, seq_hi.
constexpr size_t kHeaderLen = 5;
constexpr size_t kCrcLen = 1;

// Largest payload we define today is EstopPayload (3 bytes). Sized with a
// little headroom so adding a field later doesn't immediately require
// touching every buffer size in main.cpp.
constexpr size_t kMaxPayloadLen = 8;
constexpr size_t kMaxPacketLen = kHeaderLen + kMaxPayloadLen + kCrcLen;

enum class PacketType : uint8_t {
  kHeartbeat = 1,
  kGesture = 2,
  kEstop = 3,
  kAck = 4,
};

// --- Payloads -------------------------------------------------------------
// Packed, fixed layout, no padding: these are serialized field-by-field by
// protocol.cpp rather than memcpy'd as structs, so struct packing pragmas
// are deliberately NOT used here -- keep it that way so the struct is also
// a convenient in-memory representation on both the read and write side.

struct HeartbeatPayload {
  uint16_t battery_mv;
  uint8_t imu_ok;         // 0/1
  uint8_t uwb_ok;         // 0/1
  uint8_t estop_latched;  // 0/1 -- mirrors the latch so the drone can alarm
};
constexpr size_t kHeartbeatPayloadLen = 5;

struct GesturePayload {
  uint8_t gesture_id;  // see gestures.h GestureId
  uint8_t confidence;  // 0..255
};
constexpr size_t kGesturePayloadLen = 2;

struct EstopPayload {
  uint8_t pressed;         // 1 = asserting/latched, 0 = cleared
  uint16_t press_count;    // monotonically increasing debounced press count
};
constexpr size_t kEstopPayloadLen = 3;

// --- CRC -------------------------------------------------------------------

// CRC-8, poly 0x07, init 0x00, no reflection, no final XOR (the classic
// "CRC-8/SMBUS" variant). Table-free byte-at-a-time implementation: at 8
// bytes/packet and 5-20 Hz packet rates this is nowhere near a hot enough
// path to justify a 256-byte lookup table in flash.
uint8_t crc8(const uint8_t* data, size_t len);

// --- Encoders ----------------------------------------------------------
// Every encoder writes a complete, CRC-terminated frame into `out` and
// returns the number of bytes written, or 0 if `out_capacity` was too
// small for the frame (caller error -- buffers are all compile-time sized
// in main.cpp so this should never happen at runtime).
//
// None of these allocate; `out` is caller-owned, stack or static storage.

size_t encodeHeartbeat(uint8_t* out, size_t out_capacity, uint16_t seq,
                        const HeartbeatPayload& payload);

size_t encodeGesture(uint8_t* out, size_t out_capacity, uint16_t seq,
                      const GesturePayload& payload);

size_t encodeEstop(uint8_t* out, size_t out_capacity, uint16_t seq,
                    const EstopPayload& payload);

// --- Decoder -----------------------------------------------------------

struct DecodedHeader {
  uint8_t version;
  PacketType type;
  uint16_t seq;
  const uint8_t* payload;  // points into `data`, NOT owned/copied
  size_t payload_len;
};

// Validates magic, version, length and CRC, then fills `out`. Returns false
// on any validation failure (bad magic, bad version, truncated frame, CRC
// mismatch, or unrecognized type). Used mainly by host-side tooling and
// unit tests; the band itself is a pure transmitter today (see main.cpp),
// but the haptic characteristic write path could grow into a framed
// command in the future, so this is kept real rather than left as a stub.
bool decode(const uint8_t* data, size_t len, DecodedHeader* out);

}  // namespace protocol
