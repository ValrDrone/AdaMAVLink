with Ada.Text_IO;
with Interfaces;
with MAVLink;           use MAVLink;
with MAVLink.Frames;    use MAVLink.Frames;
with MAVLink.Heartbeat;

--  The MAVLink v2 frame layer and HEARTBEAT against frames encoded by
--  pymavlink 2.4.50 (MAVLINK20=1, dialect v20.common), an independent
--  implementation; tests/vectors/generate.py (make vectors) prints them.
procedure Test_MAVLink is
   use type Interfaces.Unsigned_8;
   use type Interfaces.Unsigned_16;
   use type Interfaces.Unsigned_32;
   use type MAVLink.Heartbeat.System_State;

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
        MAVLink.Heartbeat.Pack
          ((Custom_Mode  => 16#1234_5678#,
            Vehicle_Type => MAVLink.Heartbeat.Type_Quadrotor,
            Autopilot    => MAVLink.Heartbeat.Autopilot_Generic,
            Base_Mode    =>
              MAVLink.Heartbeat.Mode_Safety_Armed
              or MAVLink.Heartbeat.Mode_Custom_Enabled,
            State_Known  => True,
            State        => MAVLink.Heartbeat.Active,
            Version      => MAVLink.Heartbeat.Protocol_Version)),
      Frame   => Frame,
      Last    => Last);
   Check (Last = Quad'Length and then Frame (1 .. Last) = Quad,
          "heartbeat frame matches pymavlink");

   Encode
     (Head    => (Sequence => 255, System => 255, Component => 190, Id => 0,
                  Compat_Flags => 0),
      Payload =>
        MAVLink.Heartbeat.Pack
          ((Custom_Mode => 5, Vehicle_Type => 0, Autopilot => 0,
            Base_Mode   => 0, State_Known => True,
            State       => MAVLink.Heartbeat.Uninitialised, Version => 0)),
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
      H : MAVLink.Heartbeat.Content;
      M : Message;
   begin
      Check (Feed (P, Quad) = Message_Ready, "parse the quadrotor heartbeat");
      M := Received (P);
      Check
        (M.Head = (Sequence => 7, System => 1, Component => 1, Id => 0,
                   Compat_Flags => 0)
         and M.Length = 9,
         "header and length");
      H := MAVLink.Heartbeat.Unpack (M);
      Check
        (H.Custom_Mode = 16#1234_5678#
         and H.Vehicle_Type = MAVLink.Heartbeat.Type_Quadrotor
         and H.Autopilot = MAVLink.Heartbeat.Autopilot_Generic
         and H.Base_Mode = 16#81#
         and H.State_Known
         and H.State = MAVLink.Heartbeat.Active
         and H.Version = 3,
         "heartbeat fields");

      Check (Feed (P, Short) = Message_Ready, "parse a truncated heartbeat");
      M := Received (P);
      H := MAVLink.Heartbeat.Unpack (M);
      Check
        (M.Length = 1 and M.Head.System = 255 and M.Head.Component = 190
         and H.Custom_Mode = 5 and H.Vehicle_Type = 0 and H.Base_Mode = 0
         and H.State_Known and H.Version = 0,
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

   --  A message ID this build does not know (SYS_STATUS, 1).
   declare
      P       : Parser;
      Foreign : Byte_Array := Quad;
   begin
      Foreign (8) := 1;
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

   --  A system status outside MAV_STATE is unknown, not a state.
   declare
      M : Message;
   begin
      M.Head.Id := MAVLink.Heartbeat.Id;
      M.Length := 9;
      M.Payload (8) := 200;
      Check
        (not MAVLink.Heartbeat.Unpack (M).State_Known,
         "status 200 is unknown");
   end;

   Ada.Text_IO.Put_Line ("MAVLink frame and heartbeat tests passed");
end Test_MAVLink;
