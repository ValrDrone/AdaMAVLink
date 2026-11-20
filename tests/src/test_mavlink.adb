with Ada.Text_IO;
with Interfaces;
with MAVLink;           use MAVLink;
with MAVLink.Frames;    use MAVLink.Frames;
with MAVLink.Floats;
with MAVLink.Messages.Heartbeat;
with MAVLink.Wire;

--  The MAVLink v2 frame layer and HEARTBEAT against frames encoded by
--  pymavlink 2.4.50 (MAVLINK20=1, dialect v20.common), an independent
--  implementation; tests/vectors/generate.py (make vectors) prints them.
procedure Test_MAVLink is
   use type Interfaces.Unsigned_8;
   use type Interfaces.Unsigned_16;
   use type Interfaces.Unsigned_32;
   use type Interfaces.Integer_8;
   use type Interfaces.Integer_32;
   use type Interfaces.Integer_64;
   use type Interfaces.Integer_16;
   package Heartbeat renames MAVLink.Messages.Heartbeat;
   use type Heartbeat.Content;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;

   --  Sequence 7, system 1, component 1: a quadrotor, generic autopilot,
   --  base mode 16#81#, custom mode 16#12345678#, active, version 3.
   Quad : constant Byte_Array (1 .. 21) :=
     [16#FD#, 16#09#, 16#00#, 16#00#, 16#07#, 16#01#, 16#01#, 16#00#,
      16#00#, 16#00#, 16#78#, 16#56#, 16#34#, 16#12#, 16#02#, 16#00#,
      16#81#, 16#04#, 16#03#, 16#69#, 16#CB#];

   --  Sequence 255, system 255, component 190, custom mode 5, every other
   --  field zero: pymavlink truncates the payload to one byte.
   Short : constant Byte_Array (1 .. 13) :=
     [16#FD#, 16#01#, 16#00#, 16#00#, 16#FF#, 16#FF#, 16#BE#, 16#00#,
      16#00#, 16#00#, 16#05#, 16#CF#, 16#4E#];

   --  Feed Bytes; the events of all but the last byte must be None, and
   --  the last byte's event is returned.
   function Feed (P : in out Parser; Bytes : Byte_Array) return Event is
      Result : Event := None;
   begin
      for I in Bytes'Range loop
         Push (P, Bytes (I), Result);
         if I < Bytes'Last then
            Check (Result = None, "no event before a frame's last byte");
         end if;
      end loop;
      return Result;
   end Feed;

   Frame : Frame_Bytes;
   Last  : Frame_Size;
begin
   --  CRC-16/MCRF4XX check value.
   Check
     (Checksum ([16#31#, 16#32#, 16#33#, 16#34#, 16#35#,
                 16#36#, 16#37#, 16#38#, 16#39#]) = 16#6F91#,
      "CRC-16/MCRF4XX of 123456789");

   --  Encoding.
   Encode
     (Head    => (Sequence => 7, System => 1, Component => 1, Id => 0,
                  Compat_Flags => 0),
      Payload =>
        Heartbeat.Pack
          ((Custom_Mode     => 16#1234_5678#,
            Type_Field      => 2,       --  MAV_TYPE_QUADROTOR
            Autopilot       => 0,       --  MAV_AUTOPILOT_GENERIC
            Base_Mode       => 16#81#,  --  armed, custom mode enabled
            System_Status   => 4,       --  MAV_STATE_ACTIVE
            Mavlink_Version => 3)),
      Frame   => Frame,
      Last    => Last);
   Check (Last = Quad'Length and then Frame (1 .. Last) = Quad,
          "heartbeat frame matches pymavlink");

   Encode
     (Head    => (Sequence => 255, System => 255, Component => 190, Id => 0,
                  Compat_Flags => 0),
      Payload =>
        Heartbeat.Pack ((Custom_Mode => 5, others => 0)),
      Frame   => Frame,
      Last    => Last);
   Check (Last = Short'Length and then Frame (1 .. Last) = Short,
          "trailing zeros truncated as pymavlink does");

   Encode
     (Head    => (others => <>),
      Payload => [1 .. 9 => 0],
      Frame   => Frame,
      Last    => Last);
   Check (Frame (2) = 1 and Last = 13, "an all-zero payload keeps one byte");

   --  Parsing.
   declare
      P : Parser;
      H : Heartbeat.Content;
      M : Message;
   begin
      Check (Feed (P, Quad) = Message_Ready, "parse the quadrotor heartbeat");
      M := Received (P);
      Check
        (M.Head = (Sequence => 7, System => 1, Component => 1, Id => 0,
                   Compat_Flags => 0)
         and M.Length = 9,
         "header and length");
      H := Heartbeat.Unpack (M);
      Check
        (H.Custom_Mode = 16#1234_5678#
         and H.Type_Field = 2
         and H.Autopilot = 0
         and H.Base_Mode = 16#81#
         and H.System_Status = 4
         and H.Mavlink_Version = 3,
         "heartbeat fields");

      Check (Feed (P, Short) = Message_Ready, "parse a truncated heartbeat");
      M := Received (P);
      H := Heartbeat.Unpack (M);
      Check
        (M.Length = 1 and M.Head.System = 255 and M.Head.Component = 190
         and H = (Custom_Mode => 5, others => 0),
         "truncated fields read as zero");
      Check (Counts (P).Messages = 2, "two messages counted");
   end;

   --  Noise and a v1 frame before a frame are skipped.
   declare
      P : Parser;
      R : Event;
   begin
      for B of Byte_Array'([16#00#, 16#FE#, 16#09#, 16#55#]) loop
         Push (P, B, R);
         Check (R = None, "noise gives no event");
      end loop;
      Check (Feed (P, Quad) = Message_Ready, "frame after noise");
      Check (Counts (P).Skipped_Bytes = 4, "noise bytes counted");
   end;

   --  A damaged payload byte fails the checksum; the next frame parses.
   declare
      P       : Parser;
      Damaged : Byte_Array := Quad;
   begin
      Damaged (12) := Damaged (12) xor 16#01#;
      Check (Feed (P, Damaged) = Bad_Checksum, "damaged frame rejected");
      Check (Feed (P, Quad) = Message_Ready, "next frame accepted");
      Check (Counts (P).Bad_Checksums = 1, "bad checksum counted");
   end;

   --  A message ID this build does not know (SYSTEM_TIME, 2).
   declare
      P       : Parser;
      Foreign : Byte_Array := Quad;
   begin
      Foreign (8) := 2;
      Check (Feed (P, Foreign) = Unknown_Message, "unknown message ID");
      Check (Counts (P).Unknown_Messages = 1, "unknown message counted");
   end;

   --  A signed frame is read to its end and reported; the parser then
   --  takes the next frame.
   declare
      P      : Parser;
      Signed : Byte_Array (1 .. Quad'Length + Signature_Length) :=
        [others => 16#FD#];
   begin
      Signed (1 .. Quad'Length) := Quad;
      Signed (3) := Incompat_Signed;
      Check (Feed (P, Signed) = Unsupported_Flags, "signed frame reported");
      Check (Feed (P, Quad) = Message_Ready, "frame after a signed one");
   end;

   --  An incompatibility flag this build does not know.
   declare
      P       : Parser;
      Flagged : Byte_Array := Quad;
   begin
      Flagged (3) := 16#02#;
      Check (Feed (P, Flagged) = Unsupported_Flags, "unknown flag reported");
      Check (Counts (P).Unsupported_Flags = 1, "unsupported flags counted");
   end;

   --  Signed fields, both directions, at their extremes.
   declare
      B : Byte_Array (1 .. 8) := [others => 0];
      P : Payload_Bytes := [others => 0];
   begin
      MAVLink.Wire.Put_I16 (B, 1, -2);
      Check (B (1 .. 2) = [16#FE#, 16#FF#], "int16 -2");
      MAVLink.Wire.Put_I64 (B, 1, Interfaces.Integer_64'First);
      P (1 .. 8) := B;
      Check (MAVLink.Wire.I64 (P, 1) = Interfaces.Integer_64'First,
             "int64 first round trip");
      MAVLink.Wire.Put_I32 (B, 1, Interfaces.Integer_32'First);
      P (1 .. 8) := B;
      Check (MAVLink.Wire.I32 (P, 1) = Interfaces.Integer_32'First,
             "int32 first round trip");
      P (1) := 16#80#;
      Check (MAVLink.Wire.I8 (P, 1) = -128, "int8 -128");
   end;

   --  Floats: finite values convert both ways; NaN and infinity are not
   --  finite.
   Check (MAVLink.Floats.To_Bits (Float'(1.5)) = 16#3FC0_0000#, "1.5 bits");
   Check (MAVLink.Floats.To_Float (16#C120_0000#) = -10.0, "-10.0 value");
   Check (not MAVLink.Floats.Is_Finite (MAVLink.Floats.NaN), "NaN");
   Check (not MAVLink.Floats.Is_Finite (Float_Bits'(16#FF80_0000#)),
          "minus infinity");
   Check (MAVLink.Floats.To_Long_Float
            (MAVLink.Floats.To_Bits (Long_Float'(-0.25))) = -0.25,
          "double round trip");
   Check (not MAVLink.Floats.Is_Finite (MAVLink.Floats.Double_NaN),
          "double NaN");

   Ada.Text_IO.Put_Line ("MAVLink frame and heartbeat tests passed");
end Test_MAVLink;
