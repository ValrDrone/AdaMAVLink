#!/usr/bin/env python3
"""Writes AdaMAVLink's message codecs from the MAVLink XML definitions.

    adamavlink_gen.py DEFINITION SELECTION OUTPUT_DIR [DOC]

DEFINITION is the top-level XML file (definitions/common.xml); its
<include>s are read too. SELECTION lists the messages to write, one name per
line. OUTPUT_DIR receives mavlink-dialect.ads and one
mavlink-messages-<name>.ads/.adb per message; DOC, if given, a Markdown
table of them.

Python 3.9+, standard library only. Written from the MAVLink serialization
specification (mavlink.io/en/guide/serialization.html and
mavlink.io/en/guide/crc.html); it shares no code with mavgen.
"""

import re
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

# Element size in bytes, Ada type, Wire reader and writer suffix.
TYPES = {
    "uint8_t": (1, "Unsigned_8", "U8"),
    "char": (1, "Unsigned_8", "U8"),
    "int8_t": (1, "Integer_8", "I8"),
    "uint16_t": (2, "Unsigned_16", "U16"),
    "int16_t": (2, "Integer_16", "I16"),
    "uint32_t": (4, "Unsigned_32", "U32"),
    "int32_t": (4, "Integer_32", "I32"),
    "float": (4, "Float_Bits", "F32"),
    "uint64_t": (8, "Unsigned_64", "U64"),
    "int64_t": (8, "Integer_64", "I64"),
    "double": (8, "Double_Bits", "F64"),
}

ADA_RESERVED = set(
    """abort abs abstract accept access aliased all and array at begin body
    case constant declare delay delta digits do else elsif end entry exception
    exit for function generic goto if in interface is limited loop mod new not
    null of or others out overriding package parallel pragma private procedure
    protected raise range record rem renames requeue return reverse select
    separate some subtype synchronized tagged task terminate then type until
    use when while with xor""".split()
)


class Field:
    def __init__(self, element, extension):
        raw_type = element.get("type")
        self.name = element.get("name")
        self.units = element.get("units")
        self.enum = element.get("enum")
        self.description = " ".join((element.text or "").split())
        self.extension = extension
        match = re.fullmatch(r"(\w+)\[(\d+)\]", raw_type)
        if match:
            base, self.count = match.group(1), int(match.group(2))
        else:
            base, self.count = raw_type, None
        # HEARTBEAT's mavlink_version is a uint8_t the C generator fills in.
        self.crc_type = "uint8_t" if base == "uint8_t_mavlink_version" else base
        if self.crc_type not in TYPES:
            raise SystemExit(f"field {self.name}: unknown type {raw_type}")
        self.size, self.ada_type, self.wire = TYPES[self.crc_type]
        self.offset = None  # 1-based, set by Message

    @property
    def length(self):
        return self.size * (self.count or 1)

    @property
    def ada_name(self):
        name = ada_identifier(self.name)
        return name + "_Field" if name.lower() in ADA_RESERVED else name


class Message:
    def __init__(self, element, source):
        self.name = element.get("name")
        self.id = int(element.get("id"))
        self.source = source
        self.description = " ".join(
            (element.findtext("description") or "").split()
        )
        self.fields = []
        extension = False
        for child in element:
            if child.tag == "extensions":
                extension = True
            elif child.tag == "field":
                self.fields.append(Field(child, extension))
        base = [f for f in self.fields if not f.extension]
        # Wire order: the base fields by element size, largest first, keeping
        # definition order among equals (a stable sort); extensions after them
        # in definition order.
        self.wire = sorted(base, key=lambda f: f.size, reverse=True) + [
            f for f in self.fields if f.extension
        ]
        offset = 1
        for f in self.wire:
            f.offset = offset
            offset += f.length
        self.length = offset - 1
        if not 1 <= self.length <= 255:
            raise SystemExit(f"{self.name}: payload length {self.length}")
        self.crc_extra = crc_extra(self.name, self.wire[: len(base)])

    @property
    def package(self):
        return "MAVLink.Messages." + ada_identifier(self.name)

    @property
    def file_stem(self):
        return "mavlink-messages-" + self.name.lower()


def ada_identifier(name):
    return "_".join(part.capitalize() for part in name.split("_") if part)


def x25(crc, data):
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc_extra(name, base_fields):
    """CRC_EXTRA: the checksum of the message name and its base fields'
    types and names in wire order, each followed by a space, with each
    array's length as a byte; then the checksum's two bytes XORed."""
    crc = x25(0xFFFF, (name + " ").encode())
    for f in base_fields:
        crc = x25(crc, (f.crc_type + " ").encode())
        crc = x25(crc, (f.name + " ").encode())
        if f.count:
            crc = x25(crc, bytes([f.count]))
    return (crc & 0xFF) ^ (crc >> 8)


def read_messages(path, seen=None):
    seen = set() if seen is None else seen
    path = Path(path).resolve()
    if path in seen:
        return {}
    seen.add(path)
    root = ET.parse(path).getroot()
    messages = {}
    for include in root.findall("include"):
        messages.update(read_messages(path.parent / include.text.strip(), seen))
    for element in root.iter("message"):
        message = Message(element, path.name)
        if message.name in messages:
            raise SystemExit(f"{message.name} defined twice")
        messages[message.name] = message
    return messages


def read_selection(path):
    names = []
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            names.append(line)
    return names


def comment(text, indent, width=78):
    prefix = " " * indent + "--  "
    return [
        line.rstrip()
        for line in textwrap.wrap(
            text, width=width, initial_indent=prefix, subsequent_indent=prefix
        )
    ] or []


def header(message):
    return [
        "--  Generated by generator/adamavlink_gen.py from",
        f"--  definitions/{message.source}, message {message.name}. Do not edit.",
        "",
        "pragma SPARK_Mode (On);",
        "",
    ]


def field_doc(f):
    parts = []
    if f.description:
        parts.append(f.description)
    if f.units:
        parts.append(f"[{f.units}]")
    if f.enum:
        parts.append(f"({f.enum})")
    if f.ada_type in ("Float_Bits", "Double_Bits"):
        parts.append("(IEEE bits: MAVLink.Floats)")
    if f.extension:
        parts.append("Extension field: zero from senders that predate it.")
    return " ".join(parts)


def array_type(f):
    return ada_identifier(f.name) + "_Array"


def spec(message):
    lines = header(message)
    lines += [f"package {message.package} is"]
    lines += comment(message.description, 3)
    lines += [
        f"   Id     : constant Message_Id := {message.id};",
        f"   Length : constant := {message.length};",
        "",
        "   subtype Payload is Byte_Array (1 .. Length);",
        "",
    ]
    for f in message.wire:
        if f.count:
            if f.size == 1 and f.ada_type == "Unsigned_8":
                lines.append(
                    f"   subtype {array_type(f)} is Byte_Array (1 .. {f.count});"
                )
            else:
                lines.append(
                    f"   type {array_type(f)} is array (1 .. {f.count}) of "
                    f"{f.ada_type};"
                )
    if any(f.count for f in message.wire):
        lines.append("")
    lines.append("   type Content is record")
    for f in message.fields:  # definition order reads best
        lines += comment(field_doc(f), 6)
        if f.count:
            lines.append(
                f"      {f.ada_name} : {array_type(f)} := [others => 0];"
            )
        else:
            lines.append(f"      {f.ada_name} : {f.ada_type} := 0;")
    lines += [
        "   end record;",
        "",
        "   function Pack (C : Content) return Payload;",
        "",
        "   --  The content of a received message M for this ID.",
        "   function Unpack (M : Message) return Content",
        "   with Pre => M.Head.Id = Id;",
        f"end {message.package};",
        "",
    ]
    return "\n".join(lines)


def body(message):
    pack, unpack = [], []
    for f in message.wire:
        o = f.offset
        if f.count and f.size == 1 and f.ada_type == "Unsigned_8":
            last = o + f.count - 1
            pack.append(f"      Result ({o} .. {last}) := C.{f.ada_name};")
            unpack.append(
                f"      Result.{f.ada_name} := M.Payload ({o} .. {last});"
            )
        elif f.count:
            pack += [
                f"      for K in C.{f.ada_name}'Range loop",
                f"         Wire.Put_{f.wire} (Result, {o} + (K - 1) * {f.size}, "
                f"C.{f.ada_name} (K));",
                "      end loop;",
            ]
            unpack += [
                f"      for K in Result.{f.ada_name}'Range loop",
                f"         Result.{f.ada_name} (K) := Wire.{f.wire} "
                f"(M.Payload, {o} + (K - 1) * {f.size});",
                "      end loop;",
            ]
        else:
            pack.append(f"      Wire.Put_{f.wire} (Result, {o}, C.{f.ada_name});")
            unpack.append(
                f"      Result.{f.ada_name} := Wire.{f.wire} (M.Payload, {o});"
            )
    lines = header(message)
    lines += [
        "with MAVLink.Wire;",
        "",
        f"package body {message.package} is",
        "   function Pack (C : Content) return Payload is",
        "      Result : Payload := [others => 0];",
        "   begin",
        *pack,
        "      return Result;",
        "   end Pack;",
        "",
        "   function Unpack (M : Message) return Content is",
        "      Result : Content;",
        "   begin",
        *unpack,
        "      return Result;",
        "   end Unpack;",
        f"end {message.package};",
        "",
    ]
    return "\n".join(lines)


def dialect(messages, definition):
    lines = [
        "--  Generated by generator/adamavlink_gen.py from",
        f"--  definitions/{definition} and generator/messages.txt. Do not edit.",
        "",
        "pragma SPARK_Mode (On);",
        "",
        "--  The messages this build knows: their CRC_EXTRA and full payload",
        "--  length, extension fields included. A frame for a message not",
        "--  listed cannot have its checksum checked and is reported, not",
        "--  delivered.",
        "",
        "package MAVLink.Dialect is",
        "   function Info (Id : Message_Id) return Message_Info",
        "   is (case Id is",
    ]
    for m in sorted(messages, key=lambda m: m.id):
        lines.append(
            f"         when {m.id} => (Known => True, CRC_Extra => "
            f"{m.crc_extra}, Max_Length => {m.length}),  --  {m.name}"
        )
    lines += [
        "         when others => (others => <>));",
        "",
        "   function Is_Known (Id : Message_Id) return Boolean is (Info (Id).Known);",
        "end MAVLink.Dialect;",
        "",
    ]
    return "\n".join(lines)


def doc(messages, definition):
    lines = [
        "# Messages",
        "",
        f"Generated from `definitions/{definition}` by",
        "`generator/adamavlink_gen.py` (`make generate`); the selection is",
        "`generator/messages.txt`.",
        "",
        "| Message | ID | Package | Payload bytes | CRC_EXTRA |",
        "| --- | --- | --- | --- | --- |",
    ]
    for m in sorted(messages, key=lambda m: m.id):
        lines.append(
            f"| {m.name} | {m.id} | `{m.package}` | {m.length} | {m.crc_extra} |"
        )
    return "\n".join(lines) + "\n"


def write(path, text):
    path.write_text(text, newline="\n")


def main(argv):
    if len(argv) not in (4, 5):
        raise SystemExit(__doc__)
    definition, selection, output = Path(argv[1]), Path(argv[2]), Path(argv[3])
    known = read_messages(definition)
    names = read_selection(selection)
    missing = [n for n in names if n not in known]
    if missing:
        raise SystemExit(f"not in the definitions: {', '.join(missing)}")
    messages = [known[n] for n in names]
    output.mkdir(parents=True, exist_ok=True)
    for old in output.glob("mavlink-messages-*.ad?"):
        old.unlink()
    for m in messages:
        write(output / (m.file_stem + ".ads"), spec(m))
        write(output / (m.file_stem + ".adb"), body(m))
    write(output / "mavlink-dialect.ads", dialect(messages, definition.name))
    if len(argv) == 5:
        write(Path(argv[4]), doc(messages, definition.name))
    print(f"{len(messages)} messages written to {output}")


if __name__ == "__main__":
    main(sys.argv)
