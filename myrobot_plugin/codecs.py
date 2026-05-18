"""Wire codec for the example robot's UDP protocol.

The (made-up) robot speaks a tiny framed UDP protocol so this example exercises
a *non-ROS* transport — the kind of robot the class-based plugin framework was
built for. Every packet is ``<1-byte opcode><payload>``:

============ ====== =========================================
Opcode       Bytes  Payload
============ ====== =========================================
``0x00``     1      Heartbeat (no payload)
``0x01``     25     Telemetry: 3 little-endian float64 ``x, y, yaw``
``0x02``     25     Command:   3 little-endian float64 ``vx, vy, vyaw``
============ ====== =========================================
"""

import struct
from typing import Optional, Tuple

HEARTBEAT_OP = 0x00
TELEMETRY_OP = 0x01
COMMAND_OP = 0x02

_VEC3 = struct.Struct("<3d")
_PACKET_LEN = 1 + _VEC3.size  # opcode + 3 float64


def encode_heartbeat() -> bytes:
    """Build a heartbeat packet."""
    return bytes([HEARTBEAT_OP])


def encode_command(vx: float, vy: float, vyaw: float) -> bytes:
    """Build a velocity command packet."""
    return bytes([COMMAND_OP]) + _VEC3.pack(vx, vy, vyaw)


def encode_telemetry(x: float, y: float, yaw: float) -> bytes:
    """Build a telemetry packet (used by the mock robot in ``server_node.py``)."""
    return bytes([TELEMETRY_OP]) + _VEC3.pack(x, y, yaw)


def decode_telemetry(raw: bytes) -> Optional[Tuple[float, float, float]]:
    """Parse a telemetry packet into ``(x, y, yaw)``; return ``None`` for any
    packet that is not well-formed telemetry."""
    if len(raw) != _PACKET_LEN or raw[0] != TELEMETRY_OP:
        return None
    return _VEC3.unpack(raw[1:])


def decode_command(raw: bytes) -> Optional[Tuple[float, float, float]]:
    """Parse a command packet into ``(vx, vy, vyaw)`` (used by the mock robot)."""
    if len(raw) != _PACKET_LEN or raw[0] != COMMAND_OP:
        return None
    return _VEC3.unpack(raw[1:])
