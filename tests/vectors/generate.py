"""Expected frames for tests/src/test_mavlink.adb, from pymavlink.

pymavlink is an independent MAVLink implementation; this script only runs
it to produce test data (make vectors). No pymavlink code is part of the
library.
"""

import os

os.environ["MAVLINK20"] = "1"

from pymavlink.dialects.v20 import common as m  # noqa: E402
from pymavlink.generator import mavcrc  # noqa: E402


class Sink:
    def __init__(self):
        self.data = b""

    def write(self, data):
        self.data += data


def frame(sequence, system, component, message):
    sink = Sink()
    mav = m.MAVLink(sink, srcSystem=system, srcComponent=component)
    mav.seq = sequence
    mav.send(message)
    return sink.data


def ada(name, data):
    body = ", ".join("16#%02X#" % b for b in data)
    print(f"{name} ({len(data)} bytes): [{body}]")


print("CRC-16/MCRF4XX of 123456789:", hex(mavcrc.x25crc(b"123456789").crc))
print(
    "HEARTBEAT CRC_EXTRA",
    m.MAVLink_heartbeat_message.crc_extra,
    "payload length",
    m.MAVLink_heartbeat_message.unpacker.size,
)
ada(
    "Quad",
    frame(
        7, 1, 1,
        m.MAVLink_heartbeat_message(
            m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_GENERIC, 0x81, 0x12345678,
            m.MAV_STATE_ACTIVE, 3,
        ),
    ),
)
ada("Short", frame(255, 255, 190, m.MAVLink_heartbeat_message(0, 0, 0, 5, 0, 0)))
