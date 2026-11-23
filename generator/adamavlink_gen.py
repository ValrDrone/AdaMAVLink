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


class Enum:
    def __init__(self, element):
        self.name = element.get("name")
        self.bitmask = element.get("bitmask") == "true"
        self.description = " ".join(
            (element.findtext("description") or "").split()
        )
        self.entries = []
        for entry in element.findall("entry"):
            self.entries.append((entry.get("name"), int(entry.get("value"))))
        if not self.entries:
            raise SystemExit(f"enum {self.name}: no entries")
        values = [v for _, v in self.entries]
        if len(set(values)) != len(values):
            raise SystemExit(f"enum {self.name}: duplicate values")

    @property
    def ada_name(self):
        return ada_identifier(self.name)

    @property
    def is_plain(self):
        # A bitmask whose values are exactly 0..n-1 names positions, not
        # states (MAV_BOOL, MAV_MODE_FLAG_DECODE_POSITION): a plain
        # enumeration fits it better than a record of Booleans.
        values = [v for _, v in self.entries]
        return not self.bitmask or sorted(values) == list(range(len(values)))

    @property
    def width(self):
        top = max(v for _, v in self.entries)
        if top <= 0xFF:
            return "Unsigned_8"
        if top <= 0xFFFF:
            return "Unsigned_16"
        return "Unsigned_32"

    @property
    def sorted_entries(self):
        # Entries by wire value: the declaration order of a plain enum's
        # literals. The list is emitted parenthesized and without explicit
        # values: one of our toolchain builds rejects unparenthesized
        # multi-line lists and explicit values alike.
        return sorted(self.entries)


def flag_component(enum_name, entry_name):
    # The record component of a flag: the entry's name without the enum's
    # prefix, when that is a usable Ada identifier (MAV_MODE_FLAG_AUTOPILOT_
    # ENABLED becomes Autopilot_Enabled); otherwise the full name.
    stripped = ada_identifier(entry_name)
    prefix = ada_identifier(enum_name) + "_"
    if stripped.startswith(prefix):
        name = stripped[len(prefix):]
        if name and name[0].isalpha() and name.lower() not in ADA_RESERVED:
            return name
    return stripped


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


def read_definitions(path, seen=None):
    seen = set() if seen is None else seen
    path = Path(path).resolve()
    if path in seen:
        return {}, {}
    seen.add(path)
    root = ET.parse(path).getroot()
    messages, enums = {}, {}
    for include in root.findall("include"):
        sub_messages, sub_enums = read_definitions(
            path.parent / include.text.strip(), seen
        )
        messages.update(sub_messages)
        enums.update(sub_enums)
    for element in root.iter("message"):
        message = Message(element, path.name)
        if message.name in messages:
            raise SystemExit(f"{message.name} defined twice")
        messages[message.name] = message
    for element in root.iter("enum"):
        enum = Enum(element)
        if enum.name in enums:
            raise SystemExit(f"{enum.name} defined twice")
        enums[enum.name] = enum
    return messages, enums


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


def field_doc(f, enums):
    parts = []
    if f.description:
        parts.append(f.description)
    if f.units:
        parts.append(f"[{f.units}]")
    if f.enum and f.enum in enums:
        enum = enums[f.enum]
        if enum.is_plain:
            parts.append(f"(MAVLink.Enums.{enum.ada_name})")
        else:
            parts.append(f"(flags: MAVLink.Enums.{enum.ada_name})")
    if f.ada_type in ("Float_Bits", "Double_Bits"):
        parts.append("(IEEE bits: MAVLink.Floats)")
    if f.extension:
        parts.append("Extension field: zero from senders that predate it.")
    return " ".join(parts)


def array_type(f):
    return ada_identifier(f.name) + "_Array"


def spec(message, enums):
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
        lines += comment(field_doc(f, enums), 6)
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
    ]
    for f in message.fields:
        if f.count and f.crc_type == "char":
            name = f.ada_name
            lines += [
                f"   --  {name} as a String: the characters before the first",
                f"   --  NUL, whose count {name}_Length gives (0 for an empty or",
                f"   --  unterminated text).",
                f"   function {name} (C : Content) return String;",
                f"   function {name}_Length (C : Content) return Natural",
                f"   with Post => {name}_Length'Result <= C.{name}'Length;",
                "",
                "   --  Copy S into the field, NUL-padding the rest; S must",
                "   --  fit in the field.",
                f"   procedure Put_{name} (C : in out Content; S : String)",
                f"   with Pre => S'Length <= {f.count};",
                "",
            ]
    lines += [
        "   --  The content of a received message M for this ID.",
        "   function Unpack (M : Message) return Content",
        "   with Pre => M.Head.Id = Id;",
        f"end {message.package};",
        "",
    ]
    return "\n".join(lines)


def string_bodies(message):
    lines = []
    for f in message.fields:
        if not (f.count and f.crc_type == "char"):
            continue
        name, n = f.ada_name, f.count
        lines += [
            f"   function {name} (C : Content) return String is",
            f"      Result : String (1 .. {n});",
            "      L      : Natural;",
            "   begin",
            f"      for K in 1 .. {n} loop",
            f"         Result (K) := Character'Val (C.{name} (K));",
            "      end loop;",
            f"      L := {name}_Length (C);",
            "      if L = 0 then",
            '         return "";',
            "      else",
            "         return Result (1 .. L);",
            "      end if;",
            f"   end {name};",
            "",
            f"   function {name}_Length (C : Content) return Natural is",
            "      Last : Natural := 0;",
            "   begin",
            f"      for K in C.{name}'Range loop",
            f"         pragma Loop_Invariant (Last <= C.{name}'Length);",
            f"         if C.{name} (K) = 16#00# then",
            "            exit;",
            "         end if;",
            "         Last := K;",
            "      end loop;",
            "      return Last;",
            f"   end {name}_Length;",
            "",
            f"   procedure Put_{name} (C : in out Content; S : String) is",
            "   begin",
            f"      C.{name} := [others => 16#00#];",
            f"      for K in S'Range loop",
            f"         C.{name} (K - S'First + 1) := Unsigned_8'Pos (Character'Pos (S (K)));",
            "      end loop;",
            f"   end Put_{name};",
            "",
        ]
    return lines


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
        *string_bodies(message),
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


def enum_spec(enums):
    lines = [
        "--  Generated by generator/adamavlink_gen.py from",
        "--  definitions/common.xml. Do not edit.",
        "",
        "pragma SPARK_Mode (On);",
        "",
        "package MAVLink.Enums is",
        "",
    ]
    lines += comment(
        "The enumerations of the definitions as Ada types. A plain "
        "enumeration becomes a discrete type with an Is_Known_ probe and a "
        "To_ conversion; a bitmask (bitmask=\"true\" in the definitions) "
        "becomes a record of Boolean components, one per entry, with To_/"
        "From_ conversions to its wire value. An entry with value 0 names "
        "the state where no other bits are set: To_ sets its component only "
        "for wire value 0 and From_ returns 0 when it is set. Conversions "
        "take "
        "the smallest unsigned type that holds the largest value: Unsigned_8 "
        "up to 255, Unsigned_16 up to 65535, else Unsigned_32.",
        0,
    )
    lines.append("")
    used = set()

    def check(name):
        if name in used:
            raise SystemExit(f"name collision: {name}")
        used.add(name)

    for enum in sorted(enums.values(), key=lambda e: e.name):
        check(enum.ada_name)
        if enum.description:
            lines += comment(enum.description, 3)
        if enum.is_plain:
            lines.append(f"   type {enum.ada_name} is")
            entries = enum.sorted_entries
            for i, (name, value) in enumerate(entries):
                literal = ada_identifier(name)
                comma = "," if i < len(entries) - 1 else ");"
                prefix = "      (" if i == 0 else "       "
                lines.append(f"{prefix}{literal}{comma}   --  {value}")
            lines += [
                "",
                f"   --  Whether Value is one of this enumeration's values.",
                f"   function Is_Known_{enum.ada_name} (Value : {enum.width}) return Boolean;",
                "",
                f"   --  Value as the enumeration; Pre: a value this build knows.",
                f"   function To_{enum.ada_name} (Value : {enum.width}) return {enum.ada_name}",
                f"   with Pre => Is_Known_{enum.ada_name} (Value);",
                "",
            ]
            check(f"Is_Known_{enum.ada_name}")
            check(f"To_{enum.ada_name}")
        else:
            lines.append(f"   type {enum.ada_name} is record")
            components = []
            for name, value in enum.entries:
                component = flag_component(enum.name, name)
                if component in components or component.lower() in ADA_RESERVED:
                    raise SystemExit(
                        f"flag {enum.name}: bad component {component}"
                    )
                components.append(component)
                lines.append(
                    f"      {component:<32} : Boolean := False;   -- {value}"
                )
            lines += [
                "   end record;",
                "",
                f"   function To_{enum.ada_name} (Value : {enum.width}) return {enum.ada_name};",
                f"   function From_{enum.ada_name} (F : {enum.ada_name}) return {enum.width};",
                "",
            ]
            check(f"To_{enum.ada_name}")
            check(f"From_{enum.ada_name}")
    lines += ["end MAVLink.Enums;", ""]
    return "\n".join(lines)


def enum_body(enums):
    lines = [
        "--  Generated by generator/adamavlink_gen.py from",
        "--  definitions/common.xml. Do not edit.",
        "",
        "pragma SPARK_Mode (On);",
        "",
        "package body MAVLink.Enums is",
    ]
    for enum in sorted(enums.values(), key=lambda e: e.name):
        if enum.is_plain:
            entries = enum.sorted_entries
            arms = " | ".join(str(v) for _, v in entries)
            first = ada_identifier(entries[0][0])
            lines += [
                f"   function Is_Known_{enum.ada_name} (Value : {enum.width}) return Boolean is",
                "   begin",
                "      case Value is",
                f"         when {arms} =>",
                "            return True;",
                "         when others =>",
                "            return False;",
                "      end case;",
                f"   end Is_Known_{enum.ada_name};",
                "",
                f"   function To_{enum.ada_name} (Value : {enum.width}) return {enum.ada_name} is",
                f"      Result : {enum.ada_name} := {first};",
                "   begin",
                "      case Value is",
            ]
            for name, value in entries:
                literal = ada_identifier(name)
                lines += [
                    f"         when {value} =>",
                    f"            Result := {literal};",
                ]
            lines += [
                "         when others =>",
                "            null;",
                "      end case;",
                "      return Result;",
                f"   end To_{enum.ada_name};",
                "",
            ]
        else:
            bit_lines = []
            for name, value in enum.entries:
                component = flag_component(enum.name, name)
                if value == 0:
                    bit_lines.append(f"      {component} => Value = 0,")
                else:
                    bit_lines.append(
                        f"      {component} => (Value and 16#{value:04X}#) /= 16#00#,"
                    )
            lines += [
                f"   function To_{enum.ada_name} (Value : {enum.width}) return {enum.ada_name} is",
                "   begin",
                "      return (",
            ]
            for i, line in enumerate(bit_lines):
                if i < len(bit_lines) - 1:
                    lines.append(line)
                else:
                    lines.append(line.rstrip(",") + ");")
            lines += [f"   end To_{enum.ada_name};", ""]
            lines += [
                f"   function From_{enum.ada_name} (F : {enum.ada_name}) return {enum.width} is",
                f"      Result : {enum.width} := 0;",
                "   begin",
            ]
            for name, value in enum.entries:
                component = flag_component(enum.name, name)
                if value == 0:
                    lines += [
                        f"         if F.{component} then",
                        "            return 0;",
                        "         end if;",
                    ]
                else:
                    lines += [
                        f"         if F.{component} then",
                        f"            Result := Result or 16#{value:04X}#;",
                        "         end if;",
                    ]
            lines += [
                "         return Result;",
                f"   end From_{enum.ada_name};",
                "",
            ]
    lines += ["end MAVLink.Enums;", ""]
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
    known, enums = read_definitions(definition)
    names = read_selection(selection)
    missing = [n for n in names if n not in known]
    if missing:
        raise SystemExit(f"not in the definitions: {', '.join(missing)}")
    messages = [known[n] for n in names]
    output.mkdir(parents=True, exist_ok=True)
    for old in output.glob("mavlink-messages-*.ad?"):
        old.unlink()
    for m in messages:
        write(output / (m.file_stem + ".ads"), spec(m, enums))
        write(output / (m.file_stem + ".adb"), body(m))
    write(output / "mavlink-dialect.ads", dialect(messages, definition.name))
    write(output / "mavlink-enums.ads", enum_spec(enums))
    write(output / "mavlink-enums.adb", enum_body(enums))
    if len(argv) == 5:
        write(Path(argv[4]), doc(messages, definition.name))
    print(f"{len(messages)} messages, {len(enums)} enums written to {output}")


if __name__ == "__main__":
    main(sys.argv)
