pragma SPARK_Mode (On);

--  MAVLink over CRSF: the CRSF MAVLink envelope, frame type 16#AA# in the
--  TBS CRSF specification (github.com/tbs-fpv/tbs-crsf-spec, crsf.md). A
--  MAVLink frame longer than a CRSF frame can hold is split into chunks,
--  each carried by one envelope whose payload (after the CRSF type byte,
--  before the CRSF CRC) is
--
--    last chunk index * 16 + this chunk's index | data size | data
--
--  with both indices counted from 0 and at most 58 data bytes. A frame of
--  58 bytes or fewer is one envelope with indices 0 and 0. The envelope
--  carries no stream or frame number, so chunks of one frame must arrive in
--  order and without chunks of another frame between them.
--
--  This package builds and joins envelope payloads. The CRSF frame around
--  them (address, length, type, CRC-8) is the caller's, as is passing a
--  joined frame to MAVLink.Frames. It does not check MAVLink checksums.

package MAVLink.Envelope is
   CRSF_Frame_Type : constant Unsigned_8 := 16#AA#;

   Max_Data     : constant := 58;
   --  The header byte, the size byte and the data.
   Max_Envelope : constant := 2 + Max_Data;
   --  The chunk indices are four bits each.
   Max_Index    : constant := 15;

   subtype Envelope_Bytes is Byte_Array (1 .. Max_Envelope);
   subtype Envelope_Size is Natural range 3 .. Max_Envelope;

   subtype Frame_Buffer is Byte_Array (1 .. Max_Frame);
   subtype Frame_Length is Positive range 1 .. Max_Frame;

   --  The largest MAVLink frame takes 5 envelopes.
   Max_Chunks : constant := (Max_Frame + Max_Data - 1) / Max_Data;
   subtype Chunk_Count is Positive range 1 .. Max_Chunks;

   --  The envelopes a frame of Length bytes takes.
   function Chunks (Length : Frame_Length) return Chunk_Count
   is ((Length + Max_Data - 1) / Max_Data);

   --  The payload of envelope Index (from 0) of Frame:
   --  Envelope (1 .. Last). Every chunk but the last is full.
   procedure Split
     (Frame    : Byte_Array;
      Index    : Natural;
      Envelope : out Envelope_Bytes;
      Last     : out Envelope_Size)
   with
     Pre  =>
       Frame'Length in Frame_Length
       and then Index < Chunks (Frame'Length),
     Post =>
       Last
       = 2 + (if Index + 1 < Chunks (Frame'Length) then Max_Data
              else Frame'Length - Index * Max_Data);

   --  What an envelope did.
   type Event is
     (None,
      --  It completed a frame: Joined (J) holds it until the next
      --  envelope.
      Frame_Ready,
      --  Its chunk did not follow the previous one (lost, repeated or
      --  reordered chunks): it is dropped with any partial frame.
      Out_Of_Order,
      --  It is shorter than its size byte says, its size is 0 or above
      --  58, its index is past its last index, or the frame would be
      --  longer than a MAVLink frame: it is dropped with any partial
      --  frame.
      Malformed);

   type Count is mod 2**32;

   type Statistics is record
      Frames    : Count := 0;
      --  Envelopes dropped as Out_Of_Order or Malformed.
      Dropped   : Count := 0;
      --  Partial frames given up: by a dropped envelope, or by a new
      --  frame's first chunk.
      Abandoned : Count := 0;
   end record;

   type Joiner is private;

   --  Take the payload of one envelope (after the CRSF type byte, before
   --  the CRSF CRC).
   procedure Join (J : in out Joiner; Envelope : Byte_Array; Result : out Event)
   with Pre => Envelope'Length <= Max_Envelope;

   --  The frame the last Frame_Ready completed: Joined (J) (1 ..
   --  Joined_Length (J)). 0 when there is none.
   function Joined_Length (J : Joiner) return Natural
   with Post => Joined_Length'Result <= Max_Frame;
   function Joined (J : Joiner) return Frame_Buffer;

   function Counts (J : Joiner) return Statistics;

   --  Forget any partial frame and the counts, as after the link was lost.
   procedure Reset (J : out Joiner);

private
   subtype Index_Value is Natural range 0 .. Max_Index;

   type Joiner is record
      --  Chunks of a frame are arriving: Buffer (1 .. Length) so far, the
      --  next chunk is Next, the frame's last chunk Final.
      Active : Boolean := False;
      Next   : Index_Value := 0;
      Final  : Index_Value := 0;
      Length : Natural range 0 .. Max_Frame := 0;
      --  The last Frame_Ready's frame is Buffer (1 .. Ready).
      Ready  : Natural range 0 .. Max_Frame := 0;
      Buffer : Frame_Buffer := [others => 0];
      Stats  : Statistics;
   end record;

   function Joined_Length (J : Joiner) return Natural is (J.Ready);
   function Joined (J : Joiner) return Frame_Buffer is (J.Buffer);
   function Counts (J : Joiner) return Statistics is (J.Stats);
end MAVLink.Envelope;
