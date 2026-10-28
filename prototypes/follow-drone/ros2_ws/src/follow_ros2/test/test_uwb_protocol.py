"""Unit tests for follow_ros2.uwb_protocol -- no ROS required to run these.

Run directly with:
    cd ros2_ws/src/follow_ros2 && python3 -m pytest test/test_uwb_protocol.py -v
"""
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from follow_ros2.uwb_protocol import (  # noqa: E402
    MAGIC,
    VERSION,
    Ack,
    Estop,
    FrameError,
    FrameReader,
    Gesture,
    GestureId,
    Heartbeat,
    PacketType,
    RangeReport,
    crc8,
    parse_frame,
)


def _build_frame(ptype: int, seq: int, payload: bytes) -> bytes:
    body = struct.pack("<BBBH", MAGIC, VERSION, ptype, seq) + payload
    return body + bytes([crc8(body)])


def _heartbeat_frame(seq=1, battery_mv=4100, imu_ok=1, uwb_ok=1, estop_latched=0):
    payload = struct.pack("<HBBB", battery_mv, imu_ok, uwb_ok, estop_latched)
    return _build_frame(PacketType.HEARTBEAT, seq, payload)


def _gesture_frame(seq=2, gesture_id=GestureId.FOLLOW, confidence=200):
    payload = struct.pack("<BB", gesture_id, confidence)
    return _build_frame(PacketType.GESTURE, seq, payload)


def _estop_frame(seq=3, pressed=1, press_count=7):
    payload = struct.pack("<BH", pressed, press_count)
    return _build_frame(PacketType.ESTOP, seq, payload)


def _ack_frame(seq=4):
    return _build_frame(PacketType.ACK, seq, b"")


def _range_frame(seq=5, tag_id=1, range_m=2.5, quality=200):
    payload = struct.pack("<HfB", tag_id, range_m, quality)
    return _build_frame(PacketType.RANGE, seq, payload)


# -- CRC-8 --------------------------------------------------------------


def test_crc8_known_vector():
    # CRC-8/SMBUS-style, poly 0x07 init 0x00, over an empty message is 0.
    assert crc8(b"") == 0
    # Deterministic, reproducible for a fixed input.
    assert crc8(b"\x00") == 0
    assert crc8(b"\x01") == 0x07


def test_crc8_changes_with_any_bit_flip():
    base = bytes([0xFB, 0x01, 0x01, 0x02, 0x00])
    base_crc = crc8(base)
    for i in range(len(base)):
        for bit in range(8):
            mutated = bytearray(base)
            mutated[i] ^= 1 << bit
            assert crc8(bytes(mutated)) != base_crc


# -- parse_frame ----------------------------------------------------------


def test_parse_heartbeat():
    frame = _heartbeat_frame(seq=42, battery_mv=3990, imu_ok=1, uwb_ok=0, estop_latched=0)
    pkt = parse_frame(frame)
    assert isinstance(pkt, Heartbeat)
    assert pkt.seq == 42
    assert pkt.battery_mv == 3990
    assert pkt.imu_ok is True
    assert pkt.uwb_ok is False
    assert pkt.estop_latched is False


def test_parse_gesture():
    frame = _gesture_frame(gesture_id=GestureId.ORBIT, confidence=99)
    pkt = parse_frame(frame)
    assert isinstance(pkt, Gesture)
    assert pkt.gesture_id == GestureId.ORBIT
    assert pkt.confidence == 99


def test_parse_estop():
    frame = _estop_frame(pressed=1, press_count=3)
    pkt = parse_frame(frame)
    assert isinstance(pkt, Estop)
    assert pkt.pressed is True
    assert pkt.press_count == 3


def test_parse_ack():
    frame = _ack_frame(seq=9)
    pkt = parse_frame(frame)
    assert isinstance(pkt, Ack)
    assert pkt.seq == 9


def test_parse_range():
    frame = _range_frame(tag_id=3, range_m=4.25, quality=180)
    pkt = parse_frame(frame)
    assert isinstance(pkt, RangeReport)
    assert pkt.tag_id == 3
    assert abs(pkt.range_m - 4.25) < 1e-5
    assert pkt.quality == 180


def test_bad_magic_rejected():
    frame = bytearray(_heartbeat_frame())
    frame[0] = 0xAA
    try:
        parse_frame(bytes(frame))
        assert False, "expected FrameError"
    except FrameError:
        pass


def test_bad_version_rejected():
    frame = bytearray(_heartbeat_frame())
    frame[1] = 2
    try:
        parse_frame(bytes(frame))
        assert False, "expected FrameError"
    except FrameError:
        pass


def test_corrupted_crc_rejected():
    frame = bytearray(_heartbeat_frame())
    frame[-1] ^= 0xFF
    try:
        parse_frame(bytes(frame))
        assert False, "expected FrameError"
    except FrameError:
        pass


def test_truncated_frame_rejected():
    frame = _heartbeat_frame()[:-2]
    try:
        parse_frame(frame)
        assert False, "expected FrameError"
    except FrameError:
        pass


def test_wrong_length_for_type_rejected():
    # A heartbeat-typed header but gesture-sized payload+crc.
    frame = _gesture_frame()
    frame = bytes([frame[0], frame[1], PacketType.HEARTBEAT]) + frame[3:]
    try:
        parse_frame(frame)
        assert False, "expected FrameError"
    except FrameError:
        pass


# -- FrameReader (streaming / resync) --------------------------------------


def test_reader_single_frame():
    r = FrameReader()
    r.feed(_gesture_frame(gesture_id=GestureId.STOP))
    pkts = r.read_all()
    assert len(pkts) == 1
    assert isinstance(pkts[0], Gesture)
    assert pkts[0].gesture_id == GestureId.STOP


def test_reader_multiple_frames_back_to_back():
    r = FrameReader()
    r.feed(_heartbeat_frame(seq=1) + _gesture_frame(seq=2) + _estop_frame(seq=3))
    pkts = r.read_all()
    assert [type(p) for p in pkts] == [Heartbeat, Gesture, Estop]
    assert [p.seq for p in pkts] == [1, 2, 3]


def test_reader_incremental_feed():
    r = FrameReader()
    frame = _heartbeat_frame(seq=5)
    # Feed one byte at a time -- the reader must not emit a partial packet.
    for i in range(len(frame) - 1):
        r.feed(frame[i : i + 1])
        assert r.read_all() == []
    r.feed(frame[-1:])
    pkts = r.read_all()
    assert len(pkts) == 1
    assert pkts[0].seq == 5


def test_reader_resyncs_after_garbage():
    r = FrameReader()
    garbage = b"\x00\x11\x22\xFB\x99"  # includes a stray magic-like byte
    r.feed(garbage + _gesture_frame(seq=7))
    pkts = r.read_all()
    assert len(pkts) == 1
    assert isinstance(pkts[0], Gesture)
    assert pkts[0].seq == 7
    assert r.errors >= 0  # garbage before the real frame must not raise


def test_reader_resyncs_after_corrupted_frame():
    r = FrameReader()
    bad = bytearray(_heartbeat_frame(seq=1))
    bad[-1] ^= 0xFF  # corrupt CRC
    r.feed(bytes(bad) + _gesture_frame(seq=2))
    pkts = r.read_all()
    assert len(pkts) == 1
    assert isinstance(pkts[0], Gesture)
    assert pkts[0].seq == 2
    assert r.errors == 1


def test_reader_handles_unknown_type_byte():
    r = FrameReader()
    unknown = struct.pack("<BBBH", MAGIC, VERSION, 0xEE, 1)
    r.feed(unknown + _ack_frame(seq=8))
    pkts = r.read_all()
    assert len(pkts) == 1
    assert isinstance(pkts[0], Ack)
    assert pkts[0].seq == 8
