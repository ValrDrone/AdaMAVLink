with Ada.Text_IO;
with Interfaces;
with MAVLink;                     use MAVLink;
with MAVLink.Envelope;            use MAVLink.Envelope;
with MAVLink.Frames;
with MAVLink.Messages.Statustext;

--  The CRSF MAVLink envelope (MAVLink.Envelope): the chunk layout of the
--  TBS CRSF specification (high nibble the last chunk's index, low nibble
--  this chunk's, then the data size), frames split and joined back, and
--  lost, repeated and malformed envelopes.
procedure Test_Envelope is
   use type Interfaces.Unsigned_8;
   use type Frames.Event;
   package Statustext renames MAVLink.Messages.Statustext;
   use type Statustext.Content;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;

   --  pymavlink's HEARTBEAT frame from test_mavlink.adb: 21 bytes.
   Quad : constant Byte_Array (1 .. 21) :=
     [16#FD#, 16#09#, 16#00#, 16#00#, 16#07#, 16#01#, 16#01#, 16#00#,
      16#00#, 16#00#, 16#78#, 16#56#, 16#34#, 16#12#, 16#02#, 16#00#,
      16#81#, 16#04#, 16#03#, 16#69#, 16#CB#];

   --  Join every envelope of Frame in order; the last must complete it and
   --  give Frame back.
   procedure Round_Trip (J : in out Joiner; Frame : Byte_Array; Name : String)
   is
      E      : Envelope_Bytes;
      Last   : Envelope_Size;
      Result : MAVLink.Envelope.Event;
      Total  : constant Chunk_Count := Chunks (Frame'Length);
   begin
      for I in 0 .. Total - 1 loop
         Split (Frame, I, E, Last);
         Check (E (1) = Interfaces.Unsigned_8 ((Total - 1) * 16 + I),
                Name & ": chunk header");
         Join (J, E (1 .. Last), Result);
         if I < Total - 1 then
            Check (Result = None, Name & ": no event before the last chunk");
         else
            Check (Result = Frame_Ready, Name & ": frame ready");
         end if;
      end loop;
      Check (Joined_Length (J) = Frame'Length, Name & ": joined length");
      Check (Joined (J) (1 .. Joined_Length (J)) = Frame, Name & ": bytes");
   end Round_Trip;

   --  Push the joined frame into a parser: it must be a message.
   function Parse (J : Joiner) return Frames.Parser is
      P      : Frames.Parser;
      Result : Frames.Event := Frames.None;
   begin
      for I in 1 .. Joined_Length (J) loop
         Frames.Push (P, Joined (J) (I), Result);
      end loop;
      Check (Result = Frames.Message_Ready, "joined frame parses");
      return P;
   end Parse;

   J      : Joiner;
   E      : Envelope_Bytes;
   Last   : Envelope_Size;
   Result : MAVLink.Envelope.Event;

   Text : Statustext.Content;
   Long : Frames.Frame_Bytes;
   Long_Last : Frames.Frame_Size;
begin
   --  The layout, as uf-crsf 0.6.1 writes it (its test: last index 5,
   --  this index 2, data 1 2 3 4 gives 52 04 01 02 03 04), here with three
   --  short chunks a sender may choose: last index 2, so 16#2x#.
   Join (J, [16#20#, 2, 16#FD#, 16#01#], Result);
   Check (Result = None, "short chunk 0");
   Join (J, [16#21#, 2, 16#00#, 16#00#], Result);
   Check (Result = None, "short chunk 1");
   Join (J, [16#22#, 1, 16#07#], Result);
   Check (Result = Frame_Ready, "short chunk 2 completes the frame");
   Check (Joined (J) (1 .. 5) = [16#FD#, 16#01#, 16#00#, 16#00#, 16#07#],
          "short chunks joined in order");

   --  A frame of 58 bytes or fewer is one envelope, indices 0 and 0.
   Split (Quad, 0, E, Last);
   Check (Last = 23 and then E (1) = 0 and then E (2) = 21
          and then E (3 .. 23) = Quad, "heartbeat envelope");
   Round_Trip (J, Quad, "heartbeat");
   declare
      P : constant Frames.Parser := Parse (J);
   begin
      Check (Frames.Received (P).Head.Id = 0, "heartbeat parsed");
   end;

   --  A full STATUSTEXT frame is 66 bytes: 58 and 8.
   Text.Severity := 4;
   for I in Text.Text'Range loop
      Text.Text (I) := Interfaces.Unsigned_8 (64 + I);
   end loop;
   Text.Id := 16#1234#;
   Text.Chunk_Seq := 9;
   Frames.Encode
     ((Sequence => 3, System => 1, Component => 1, Id => Statustext.Id,
       Compat_Flags => 0),
      Statustext.Pack (Text), Long, Long_Last);
   Check (Long_Last = 66, "statustext frame length");
   Check (Chunks (Long_Last) = 2, "statustext takes two envelopes");
   Split (Long (1 .. Long_Last), 0, E, Last);
   Check (E (1) = 16#10# and then E (2) = 58 and then Last = 60,
          "statustext chunk 0");
   Split (Long (1 .. Long_Last), 1, E, Last);
   Check (E (1) = 16#11# and then E (2) = 8 and then Last = 10,
          "statustext chunk 1");
   Round_Trip (J, Long (1 .. Long_Last), "statustext");
   Check (Statustext.Unpack (Frames.Received (Parse (J))) = Text,
          "statustext content");

   --  The largest frame (signed, full payload) takes five envelopes.
   declare
      Big : Byte_Array (1 .. Max_Frame);
   begin
      for I in Big'Range loop
         Big (I) := Interfaces.Unsigned_8 (I mod 256);
      end loop;
      Check (Chunks (Max_Frame) = 5, "largest frame takes five envelopes");
      Split (Big, 4, E, Last);
      Check (E (1) = 16#44# and then E (2) = 48, "largest frame chunk 4");
      Round_Trip (J, Big, "largest frame");
      --  Index from 1 works too.
      declare
         Shifted : constant Byte_Array (101 .. 100 + Max_Frame) := Big;
      begin
         Round_Trip (J, Shifted, "shifted bounds");
      end;
   end;

   --  A frame's first chunk while another is partial: the partial one is
   --  abandoned and the new one goes on.
   Reset (J);
   Split (Long (1 .. Long_Last), 0, E, Last);
   Join (J, E (1 .. Last), Result);
   Check (Result = None, "statustext chunk 0 alone");
   Round_Trip (J, Quad, "heartbeat after a lost chunk");
   Check (Counts (J).Abandoned = 1, "partial frame abandoned");
   Check (Counts (J).Frames = 1, "one frame");

   --  A chunk with no frame begun.
   Split (Long (1 .. Long_Last), 1, E, Last);
   Join (J, E (1 .. Last), Result);
   Check (Result = Out_Of_Order, "chunk 1 with no frame begun");
   Check (Joined_Length (J) = 0, "no frame after a dropped chunk");

   --  A repeated chunk drops the frame; the chunk after it is then out of
   --  order as well.
   declare
      Big : constant Byte_Array (1 .. 150) := [others => 16#55#];
   begin
      Split (Big, 0, E, Last);
      Join (J, E (1 .. Last), Result);
      Split (Big, 1, E, Last);
      Join (J, E (1 .. Last), Result);
      Check (Result = None, "chunk 1 of 3");
      Join (J, E (1 .. Last), Result);
      Check (Result = Out_Of_Order, "chunk 1 repeated");
      Split (Big, 2, E, Last);
      Join (J, E (1 .. Last), Result);
      Check (Result = Out_Of_Order, "chunk 2 after the frame is dropped");
      Round_Trip (J, Big, "after a repeated chunk");
   end;

   --  Malformed envelopes.
   Join (J, [16#00#, 0, 16#FD#], Result);
   Check (Result = Malformed, "data size 0");
   Join (J, [16#00#, 59] & [1 .. 58 => 16#FD#], Result);
   Check (Result = Malformed, "data size 59");
   Join (J, [16#00#, 4, 1, 2, 3], Result);
   Check (Result = Malformed, "shorter than its size");
   Join (J, [16#12#, 1, 1], Result);
   Check (Result = Malformed, "index past the last index");
   Join (J, [16#00#, 1], Result);
   Check (Result = Malformed, "no data");
   --  Five full chunks (290 bytes) would exceed the largest MAVLink frame.
   declare
      Chunk : Byte_Array (1 .. Max_Envelope) := [others => 16#33#];
   begin
      for I in 0 .. 3 loop
         Chunk (1) := Interfaces.Unsigned_8 (16#F0# + I);
         Chunk (2) := Max_Data;
         Join (J, Chunk, Result);
         Check (Result = None, "long frame chunk");
      end loop;
      Chunk (1) := 16#F4#;
      Join (J, Chunk, Result);
      Check (Result = Malformed, "frame longer than a MAVLink frame");
   end;
   Check (Counts (J).Dropped = 9, "dropped envelopes counted");
   Round_Trip (J, Quad, "heartbeat after malformed envelopes");

   Ada.Text_IO.Put_Line ("MAVLink envelope tests passed");
end Test_Envelope;
