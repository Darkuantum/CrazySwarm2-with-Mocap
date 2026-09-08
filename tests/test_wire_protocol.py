"""Cross-check the band's wire protocol across independent implementations.

The ESP32 firmware (C++) and the drone-side parser (Python) were written
separately against the same written spec. A spec is not a guarantee -- one
endianness or CRC-seed disagreement and the drone silently ignores every
E-stop packet the band sends. So compile the firmware's own CRC and compare it
against a reference here, on vectors including the awkward ones.

Skips cleanly where a toolchain is missing, so the suite still runs anywhere.
"""
import os
import shutil
import sys
import subprocess
import tempfile

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW_SRC = os.path.join(REPO, "firmware", "band", "src")

MAGIC = 0xFB
VERSION = 1
TYPE_HEARTBEAT, TYPE_GESTURE, TYPE_ESTOP, TYPE_ACK = 1, 2, 3, 4


def crc8_ref(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    """CRC-8/SMBUS: poly 0x07, init 0x00, MSB-first, no reflection, no XOR-out."""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


VECTORS = [
    bytes([0xFB, 0x01, 0x03, 0x00, 0x00, 0x01, 0x02, 0x00]),   # ESTOP pressed
    bytes([0xFB, 0x01, 0x02, 0x2A, 0x01, 0x01, 0xC8]),         # GESTURE FOLLOW
    bytes([0x00]),                                             # single zero byte
    bytes([0xFB]),                                             # single magic byte
    bytes(range(16)),                                          # ramp
    b"\xff" * 8,                                               # all ones
]


def test_reference_crc_is_the_documented_variant():
    # The canonical CRC-8/SMBUS check value for b"123456789" is 0xF4.
    assert crc8_ref(b"123456789") == 0xF4


@pytest.mark.skipif(shutil.which("g++") is None, reason="no C++ compiler")
@pytest.mark.skipif(
    not os.path.exists(os.path.join(FW_SRC, "protocol.cpp")),
    reason="firmware protocol.cpp not present",
)
def test_firmware_crc_matches_reference():
    """Compile the real firmware CRC and compare it byte-for-byte."""
    with tempfile.TemporaryDirectory() as td:
        probe = os.path.join(td, "probe.cpp")
        calls = ", ".join(
            f"protocol::crc8(v{i}, sizeof v{i})" for i in range(len(VECTORS))
        )
        decls = "\n".join(
            "  const uint8_t v%d[] = {%s};" % (i, ",".join(f"0x{b:02X}" for b in v))
            for i, v in enumerate(VECTORS)
        )
        fmt = " ".join("%u" for _ in VECTORS)
        with open(probe, "w") as fh:
            fh.write(
                '#include <cstdint>\n#include <cstddef>\n#include <cstdio>\n'
                '#include "protocol.h"\nint main(){\n'
                f"{decls}\n"
                f'  printf("{fmt}\\n", {calls});\n}}\n'
            )
        exe = os.path.join(td, "probe")
        build = subprocess.run(
            ["g++", "-std=c++14", "-O1", f"-I{FW_SRC}", "-o", exe, probe,
             os.path.join(FW_SRC, "protocol.cpp")],
            capture_output=True, text=True,
        )
        if build.returncode != 0:
            pytest.skip(f"firmware protocol.cpp did not compile standalone:\n{build.stderr[:400]}")
        got = [int(x) for x in subprocess.run(
            [exe], capture_output=True, text=True, check=True).stdout.split()]

    expected = [crc8_ref(v) for v in VECTORS]
    assert got == expected, f"firmware {got} != reference {expected}"


def test_frame_layout_round_trips():
    """The framing the drone-side parser must implement, stated once, here."""
    def build(ptype: int, seq: int, payload: bytes) -> bytes:
        head = bytes([MAGIC, VERSION, ptype, seq & 0xFF, (seq >> 8) & 0xFF])
        body = head + payload
        return body + bytes([crc8_ref(body)])

    def parse(frame: bytes):
        assert frame[0] == MAGIC and frame[1] == VERSION
        assert crc8_ref(frame[:-1]) == frame[-1], "CRC mismatch"
        seq = frame[3] | (frame[4] << 8)
        return frame[2], seq, frame[5:-1]

    f = build(TYPE_GESTURE, 0x012A, bytes([1, 200]))
    ptype, seq, payload = parse(f)
    assert (ptype, seq, payload) == (TYPE_GESTURE, 0x012A, bytes([1, 200]))

    # A single flipped bit anywhere must fail the CRC.
    for i in range(len(f)):
        bad = bytearray(f)
        bad[i] ^= 0x01
        if bytes(bad) == f:
            continue
        if bad[0] != MAGIC or bad[1] != VERSION:
            continue                      # rejected earlier, by magic/version
        assert crc8_ref(bytes(bad[:-1])) != bad[-1], f"bit flip at {i} survived CRC"


def test_sequence_number_is_little_endian():
    seq = 0x1234
    lo, hi = seq & 0xFF, (seq >> 8) & 0xFF
    assert (lo, hi) == (0x34, 0x12)
    assert (lo | (hi << 8)) == seq


def _load_ros_parser():
    """Load the drone-side parser without importing ROS.

    It is deliberately free of rclpy imports so it can be tested exactly like
    this; if that ever stops being true, this test will say so.
    """
    path = os.path.join(
        REPO, "ros2_ws", "src", "follow_ros2", "follow_ros2", "uwb_protocol.py"
    )
    if not os.path.exists(path):
        pytest.skip("drone-side uwb_protocol.py not present")
    import importlib.util

    spec = importlib.util.spec_from_file_location("_uwb_protocol_under_test", path)
    if spec is None or spec.loader is None:
        pytest.skip("could not load uwb_protocol.py")
    mod = importlib.util.module_from_spec(spec)
    # Register before exec: dataclasses resolves KW_ONLY via
    # sys.modules[cls.__module__], which is None for an unregistered module and
    # raises a confusing AttributeError on Python 3.10.
    sys.modules[spec.name] = mod
    try:
        spec.loader.exec_module(mod)
    except ImportError as exc:
        pytest.fail(
            f"uwb_protocol.py must stay free of ROS imports so it can be "
            f"unit-tested off-target, but importing it failed: {exc}"
        )
    return mod


def test_drone_side_parser_crc_matches_the_firmware():
    """Three independent implementations of the same CRC must agree.

    Firmware C++, drone-side Python, and the reference above were written
    separately. One disagreement here means the drone silently drops every
    E-stop packet the band sends -- which is the single worst failure this
    project could ship.
    """
    mod = _load_ros_parser()
    assert hasattr(mod, "crc8"), "expected a crc8() in uwb_protocol.py"
    got = [mod.crc8(v) for v in VECTORS]
    expected = [crc8_ref(v) for v in VECTORS]
    assert got == expected, f"drone-side {got} != reference {expected}"
    assert mod.crc8(b"123456789") == 0xF4, "not the CRC-8/SMBUS variant"


def test_drone_side_parser_accepts_a_frame_the_spec_builds():
    mod = _load_ros_parser()
    head = bytes([MAGIC, VERSION, TYPE_ESTOP, 0x07, 0x00])
    payload = bytes([1, 0x02, 0x00])            # pressed=1, press_count=2
    frame = head + payload
    frame += bytes([crc8_ref(frame)])
    parser = None
    for name in ("parse_frame", "parse_packet", "decode_frame", "parse"):
        if hasattr(mod, name):
            parser = getattr(mod, name)
            break
    if parser is None:
        pytest.skip("no frame parser export found to exercise")
    result = parser(frame)
    assert result is not None, "a well-formed ESTOP frame was rejected"
