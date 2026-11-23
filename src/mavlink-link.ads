pragma SPARK_Mode (On);

--  Link-layer conveniences: sender identity and sequence numbering for
--  outgoing messages, and a static handler table for incoming ones.
--
--  The Frames parser stays with the caller; this package only stamps
--  headers and dispatches.

with MAVLink.Dialect;
with MAVLink.Frames;

package MAVLink.Link is
   --  A receiver's interest in one message ID. Handlers are called with
   --  the received message and must not raise exceptions.
   type Handler is access procedure (M : Message);

   --  One dispatch table entry. A null H leaves the ID unhandled.
   type Slot is record
      Id : Message_Id;
      H  : Handler := null;
   end record;

   --  A static dispatch table. Keep it small: Dispatch scans it linearly,
   --  and the first entry whose Id matches wins.
   type Table is array (Natural range <>) of Slot;

   procedure Dispatch (T : Table; M : Message);

   type Link is private;

   procedure Initialize (L : in out Link; System, Component : Unsigned_8);

   --  Encode a payload with this link's identity and its next sequence
   --  number, then advance the number (mod 256). Payload must be the
   --  codec's packed payload for Id.
   procedure Send
     (L       : in out Link;
      Id      : Message_Id;
      Payload : Byte_Array;
      Frame   : out Frames.Frame_Bytes;
      Last    : out Frames.Frame_Size)
   with Pre => Dialect.Is_Known (Id)
              and then Payload'First = 1
              and then Payload'Length = Dialect.Info (Id).Max_Length
              and then Payload'Length >= 1;

private
   type Link is record
      System    : Unsigned_8 := 0;
      Component : Unsigned_8 := 0;
      Sequence  : Unsigned_8 := 0;
   end record;
end MAVLink.Link;
