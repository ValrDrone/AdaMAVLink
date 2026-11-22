# AdaMAVLink

MAVLink v2 frames and messages in SPARK, proved free of run-time errors
with GNATprove. Built for flight controllers, radio links and ground
stations that must not crash on any byte sequence they receive.

Status: early. The frame layer (checksum, byte-wise parser, encoder) and
16 messages generated from the MAVLink XML definitions: HEARTBEAT, the
basic telemetry, STATUSTEXT, the command and the mission protocol messages
([messages](docs/messages.md)). More are a line each in
`generator/messages.txt` ([design](docs/design.md)). `MAVLink.Envelope`
carries frames over a CRSF serial line in the CRSF MAVLink envelope
(type `16#AA#`), next to RC channels and link statistics.

## Use

```ada
with MAVLink.Frames;
with MAVLink.Messages.Heartbeat;

P      : MAVLink.Frames.Parser;
Result : MAVLink.Frames.Event;
...
MAVLink.Frames.Push (P, Byte, Result);
if Result = MAVLink.Frames.Message_Ready
  and then MAVLink.Frames.Received (P).Head.Id
           = MAVLink.Messages.Heartbeat.Id
then
   Beat := MAVLink.Messages.Heartbeat.Unpack (MAVLink.Frames.Received (P));
end if;
```

With Alire: `alr with adamavlink` (once published), or a pin to this
repository at a commit. The library has no dependencies and needs only
`Interfaces` from the runtime and uses no tasking, allocation or I/O,
so it is meant for restricted embedded runtimes too. Any GNAT
that accepts Ada 2022 builds it; GNAT 14.2 and 16.1 are the tested ones.

`ADAMAVLINK_BUILD_MODE` is `release` (default, `-O2`) or `dev` (warnings
are errors); `ADAMAVLINK_LIBRARY_TYPE` is `static` (default),
`relocatable` or `static-pic`.

## Development

Tests and proof run in a container with the toolchain pinned in
`tests/alire.toml`:

```sh
make setup      # build the image, fetch the toolchain
make generate   # after changing generator/ or definitions/
make test
make prove
make ci         # all of it, with GNAT 14.2 too
```

## Provenance

Written from the published MAVLink specification (mavlink.io). The message
definitions in `definitions/` are the MAVLink project's, MIT-licensed
([provenance](definitions/README.md)); the generator that turns them into
Ada is this project's own. No code from other MAVLink implementations is
included. The tests' expected frames were produced by running pymavlink
(`tests/vectors/`); only its output is used.

## Licence

MIT; see [LICENSE](LICENSE).
