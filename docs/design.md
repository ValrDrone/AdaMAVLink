# Design

## Frame layer

`MAVLink` (`src/mavlink.ads`) holds the types and the checksum:
CRC-16/MCRF4XX (X.25) from `16#FFFF#`, over the frame after the start byte
up to the payload's end, then over the message's CRC_EXTRA byte.

`MAVLink.Frames` parses and encodes v2 frames:

- `Push` takes one byte at a time with bounded work per byte and keeps one
  frame. It reports `Message_Ready` for a known message with a correct
  checksum; `Received` then holds the header, the received length and the
  payload, zero after the received bytes, so a codec reads fields at fixed
  offsets whether or not the sender truncated them. It also reports
  `Bad_Checksum`, `Unknown_Message` (an ID this build does not know: its
  checksum cannot be checked) and `Unsupported_Flags` (a signed frame, read
  to its end, or an incompatibility flag it does not know), and counts each
  with the bytes skipped outside frames. v1 frames (`16#FE#`) are skipped.
- `Encode` builds an unsigned frame from a header and a full payload,
  dropping the payload's trailing zero bytes and keeping at least one, as
  the specification requires of senders.

`MAVLink.Dialect` lists the messages this build knows with their CRC_EXTRA
and full payload length. It is written by hand for now.

## Messages

| Message | ID | Package | Payload |
| --- | --- | --- | --- |
| HEARTBEAT | 0 | `MAVLink.Heartbeat` | custom_mode (uint32), type, autopilot, base_mode, system_status, mavlink_version |

A system status outside MAV_STATE unpacks as unknown (`State_Known` false),
not as a state.

## What is proved

`make prove` runs GNATprove (level 2, every check an error) over every unit
in `src`: absence of run-time errors (no index, range, overflow or division
failure, for any input byte sequence) and the stated preconditions at each
call. It does not prove functional correctness, such as that `Encode` and
`Push` agree or that the CRC is the MCRF4XX one; the tests check those
against an independent implementation.

## Verification

`tests/src/test_mavlink.adb` (`make test`) checks the CRC's standard check
value (`123456789` gives `16#6F91#`) and frames encoded by pymavlink, an
independent implementation: a full heartbeat and one truncated to a single
payload byte, both encoded and parsed. It also checks noise and v1 bytes
before a frame, a damaged payload, an unknown message ID, a signed frame,
an unknown flag, and recovery after each.

## Roadmap

1. A generator from the MAVLink XML message definitions that writes
   `MAVLink.Dialect` and one SPARK package per message (record, `Pack`,
   `Unpack`), with tests generated against pymavlink.
2. The messages a vehicle and a ground station need first: SYS_STATUS,
   ATTITUDE, GLOBAL_POSITION_INT, GPS_RAW_INT, STATUSTEXT, the mission
   protocol, COMMAND_LONG / COMMAND_INT / COMMAND_ACK, the parameter
   protocol.
3. Signature checking (optional, per link).
