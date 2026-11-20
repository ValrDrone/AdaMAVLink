pragma SPARK_Mode (On);

with MAVLink.Dialect;

--  MAVLink v2 frames: a byte-wise parser and an encoder.
--
--  The parser takes one byte at a time and keeps no more than one frame,
--  so a link driver can feed it from its receive buffer with bounded work
--  per byte. A frame that does not check out is counted and the parser
--  looks for the next start byte; it never resynchronises inside a frame
--  it has accepted the header of, as the length tells it where the frame
--  ends. MAVLink v1 frames (start byte 16#FE#) are not accepted.
--
--  Signed frames are read to their end and reported as unsupported:
--  signature checking is not implemented yet, and a frame whose signature
--  is not checked is not delivered.

package MAVLink.Frames is
   type Phase is
     (Start,
      Length,
      Incompat,
      Compat,
      Sequence,
      System,
      Component,
      Id_Low,
      Id_Middle,
      Id_High,
      Payload,
      Checksum_Low,
      Checksum_High,
      Signature);

   --  What a byte did.
   type Event is
     (None,
      --  A message for a known ID with a correct checksum is complete:
      --  Received (P) holds it until the next byte.
      Message_Ready,
      --  A frame for a known message whose checksum does not match.
      Bad_Checksum,
      --  A frame for a message ID this build does not know; its checksum
      --  cannot be checked.
      Unknown_Message,
      --  A signed frame, or one with an incompatibility flag this build
      --  does not know.
      Unsupported_Flags);

   type Count is mod 2**32;

   type Statistics is record
      Messages          : Count := 0;
      Bad_Checksums     : Count := 0;
      Unknown_Messages  : Count := 0;
      Unsupported_Flags : Count := 0;
      --  Bytes outside any frame: noise, v1 frames, or the rest of a
      --  frame whose header did not check out.
      Skipped_Bytes     : Count := 0;
   end record;

   type Parser is private;

   function Received (P : Parser) return Message;

   function Counts (P : Parser) return Statistics;

   --  Feed one byte.
   procedure Push (P : in out Parser; B : Unsigned_8; Result : out Event);

   subtype Frame_Bytes is Byte_Array (1 .. Max_Frame);
   subtype Frame_Size is Natural range Header_Length + 1 + Checksum_Length
                                       .. Header_Length + Max_Payload
                                          + Checksum_Length;

   --  The unsigned frame for a message with the header Head and the full
   --  payload Payload, its trailing zero bytes dropped (keeping at least
   --  one): Frame (1 .. Last).
   procedure Encode
     (Head    : Header;
      Payload : Byte_Array;
      Frame   : out Frame_Bytes;
      Last    : out Frame_Size)
   with
     Pre =>
       Dialect.Is_Known (Head.Id)
       and then Payload'First = 1
       and then Payload'Length = Dialect.Info (Head.Id).Max_Length
       and then Payload'Length >= 1;

private
   type Parser is record
      Phase     : Frames.Phase := Start;
      Current   : Message;
      --  Payload bytes of Current received so far.
      Count     : Payload_Length := 0;
      Incompat  : Unsigned_8 := 0;
      CRC       : Unsigned_16 := CRC_Initial;
      --  The received checksum's low byte.
      CRC_Low   : Unsigned_8 := 0;
      --  The frame's outcome, decided at its checksum and reported once
      --  its signature, if any, has been read.
      Pending   : Event := None;
      Remaining : Natural range 0 .. Signature_Length := 0;
      Stats     : Statistics;
   end record;

   function Received (P : Parser) return Message is (P.Current);

   function Counts (P : Parser) return Statistics is (P.Stats);
end MAVLink.Frames;
