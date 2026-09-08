"""UWB band wire protocol: CRC-8 and packet parser.

Deliberately has ZERO ROS imports (no rclpy, no message types) so it can be
unit-tested with plain pytest -- see ros2_ws/src/follow_ros2/test/. uwb_node.py
is the only place this module is imported from ROS code.

Wire format (all little-endian):

    byte0       magic       0xFB
    byte1       version     1
    byte2       type        1=HEARTBEAT 2=GESTURE 3=ESTOP 4=ACK
    byte3-4     seq         uint16
    byte5..     payload     (type-dependent, see PAYLOAD_SIZES)
    last byte   crc8        poly 0x07, init 0x00, over all preceding bytes

Payloads:
    HEARTBEAT: uint16 battery_mv, uint8 imu_ok, uint8 uwb_ok, uint8 estop_latched
    GESTURE:   uint8 gesture_id, uint8 confidence
    ESTOP:     uint8 pressed, uint16 press_count
    ACK:       (no defined payload; treated as zero-length)

Gesture IDs: 1=FOLLOW 2=STOP 3=ORBIT 4=LAND 5=COME
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

MAGIC = 0xFB
VERSION = 1


class PacketType(IntEnum):
    HEARTBEAT = 1
    GESTURE = 2
    ESTOP = 3
    ACK = 4


class GestureId(IntEnum):
    FOLLOW = 1
    STOP = 2
    ORBIT = 3
    LAND = 4
    COME = 5


# Fixed payload sizes in bytes, keyed by PacketType. ACK carries no defined
# payload in the spec; treated as zero-length here.
PAYLOAD_SIZES = {
    PacketType.HEARTBEAT: 2 + 1 + 1 + 1,  # battery_mv, imu_ok, uwb_ok, estop_latched
    PacketType.GESTURE: 1 + 1,  # gesture_id, confidence
    PacketType.ESTOP: 1 + 2,  # pressed, press_count
    PacketType.ACK: 0,
}

_HEADER_LEN = 5  # magic, version, type, seq(2)
_CRC_LEN = 1


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    """CRC-8, poly 0x07, init 0x00, no reflection, no xor-out.

    This is the plain/"CRC-8/SMBUS"-style bit-at-a-time implementation --
    fine at band packet rates (a few hundred bytes/s), no lookup table
    needed.
    """
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


@dataclass(frozen=True)
class Heartbeat:
    seq: int
    battery_mv: int
    imu_ok: bool
    uwb_ok: bool
    estop_latched: bool


@dataclass(frozen=True)
class Gesture:
    seq: int
    gesture_id: int
    confidence: int


@dataclass(frozen=True)
class Estop:
    seq: int
    pressed: bool
    press_count: int


@dataclass(frozen=True)
class Ack:
    seq: int


Packet = "Heartbeat | Gesture | Estop | Ack"  # type-hint alias for docs only


class FrameError(ValueError):
    """Raised for a structurally invalid frame (bad magic/version/CRC/length)."""


def _decode_payload(ptype: int, seq: int, payload: bytes):
    if ptype == PacketType.HEARTBEAT:
        battery_mv, imu_ok, uwb_ok, estop_latched = struct.unpack("<HBBB", payload)
        return Heartbeat(
            seq=seq,
            battery_mv=battery_mv,
            imu_ok=bool(imu_ok),
            uwb_ok=bool(uwb_ok),
            estop_latched=bool(estop_latched),
        )
    if ptype == PacketType.GESTURE:
        gesture_id, confidence = struct.unpack("<BB", payload)
        return Gesture(seq=seq, gesture_id=gesture_id, confidence=confidence)
    if ptype == PacketType.ESTOP:
        pressed, press_count = struct.unpack("<BH", payload)
        return Estop(seq=seq, pressed=bool(pressed), press_count=press_count)
    if ptype == PacketType.ACK:
        return Ack(seq=seq)
    raise FrameError(f"unknown packet type {ptype}")


def parse_frame(frame: bytes):
    """Parse and CRC-check one complete frame (magic..crc inclusive).

    Raises FrameError on any structural problem. Returns one of
    Heartbeat/Gesture/Estop/Ack on success.
    """
    if len(frame) < _HEADER_LEN + _CRC_LEN:
        raise FrameError(f"frame too short: {len(frame)} bytes")
    if frame[0] != MAGIC:
        raise FrameError(f"bad magic 0x{frame[0]:02X}")
    if frame[1] != VERSION:
        raise FrameError(f"unsupported version {frame[1]}")

    ptype = frame[2]
    seq = struct.unpack("<H", frame[3:5])[0]

    if ptype not in PAYLOAD_SIZES:
        raise FrameError(f"unknown packet type {ptype}")
    expected_payload_len = PAYLOAD_SIZES[ptype]
    expected_len = _HEADER_LEN + expected_payload_len + _CRC_LEN
    if len(frame) != expected_len:
        raise FrameError(
            f"bad length for type {ptype}: got {len(frame)}, want {expected_len}"
        )

    body, crc_byte = frame[:-1], frame[-1]
    computed = crc8(body)
    if computed != crc_byte:
        raise FrameError(f"CRC mismatch: got 0x{crc_byte:02X}, want 0x{computed:02X}")

    payload = frame[_HEADER_LEN : _HEADER_LEN + expected_payload_len]
    return _decode_payload(ptype, seq, payload)


class FrameReader:
    """Incremental byte-stream framer for a serial port.

    Feed it raw bytes as they arrive (`feed`); it resyncs on the magic byte,
    and yields fully-parsed, CRC-valid packets via `read_all`. Invalid frames
    are dropped (and reported via `errors`) rather than raised, because one
    corrupt byte on a live UART must not take the reader down -- it should
    resync on the next magic byte instead.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)

    def _frame_len_at(self, start: int) -> Optional[int]:
        """Length of the frame starting at `start`, or None if not enough
        buffered bytes yet to know (need the type byte and header)."""
        if len(self._buf) < start + _HEADER_LEN:
            return None
        ptype = self._buf[start + 2]
        if ptype not in PAYLOAD_SIZES:
            return None  # unknown type; caller resyncs one byte
        return _HEADER_LEN + PAYLOAD_SIZES[ptype] + _CRC_LEN

    def read_all(self):
        """Return a list of successfully parsed packets found so far,
        consuming their bytes (and any skipped junk) from the internal
        buffer."""
        out = []
        while True:
            # Resync: drop leading bytes until a magic byte or we run dry.
            idx = self._buf.find(bytes([MAGIC]))
            if idx == -1:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]

            flen = self._frame_len_at(0)
            if flen is None:
                if len(self._buf) >= _HEADER_LEN and self._buf[2] not in PAYLOAD_SIZES:
                    # Unknown type byte at a magic-looking position: it was
                    # not really a frame start. Drop the magic byte and
                    # keep resyncing rather than stalling forever.
                    del self._buf[0:1]
                    continue
                break  # need more bytes to know the length

            if len(self._buf) < flen:
                break  # full frame not buffered yet

            candidate = bytes(self._buf[:flen])
            try:
                out.append(parse_frame(candidate))
                del self._buf[:flen]
            except FrameError:
                self.errors += 1
                # Drop just the magic byte and resync -- the "frame" may
                # have been junk that merely started with 0xFB.
                del self._buf[0:1]
        return out
