pragma SPARK_Mode (On);

with Interfaces; use Interfaces;

--  MAVLink v2 (mavlink.io/en/guide/serialization.html): the types and the
--  checksum shared by the frame layer (MAVLink.Frames), the table of the
--  messages this build knows (MAVLink.Dialect) and the message codecs.
--
--  A v2 frame is the start byte 16#FD#, the payload length, the
--  incompatibility and compatibility flags, a sequence number, the sender's
--  system and component IDs, a 24-bit little-endian message ID, the
--  payload, a 16-bit checksum and, when the first incompatibility flag is
--  set, a 13-byte signature. The checksum is CRC-16/MCRF4XX (X.25) over
--  every byte after the start byte up to the payload's end, then over the
--  message's CRC_EXTRA byte, which ties the checksum to the message's
--  field layout.
--
--  Payload fields are little-endian, in the order of the message
--  definition sorted by size (largest first), extension fields last. A
--  sender drops the payload's trailing zero bytes, keeping at least one; a
--  receiver reads the missing bytes as zero.

package MAVLink is
   type Byte_Array is array (Natural range <>) of Unsigned_8;

   Start_V2         : constant Unsigned_8 := 16#FD#;
   --  Start byte, length, two flag bytes, sequence, system, component and
   --  three message ID bytes.
   Header_Length    : constant := 10;
   Checksum_Length  : constant := 2;
   Signature_Length : constant := 13;
   Max_Payload      : constant := 255;
   Max_Frame        : constant :=
     Header_Length + Max_Payload + Checksum_Length + Signature_Length;

   --  The one incompatibility flag defined: the frame carries a signature.
   Incompat_Signed : constant Unsigned_8 := 16#01#;

   type Message_Id is range 0 .. 2**24 - 1;

   --  A float or double field as its IEEE 754 bits. MAVLink uses NaN for
   --  "unknown" in many fields, which Float cannot hold in SPARK, so
   --  messages keep the bits; MAVLink.Floats converts finite ones.
   type Float_Bits is new Unsigned_32;
   type Double_Bits is new Unsigned_64;

   subtype Payload_Length is Natural range 0 .. Max_Payload;
   subtype Payload_Bytes is Byte_Array (1 .. Max_Payload);

   --  CRC-16/MCRF4XX: the X.25 polynomial, reflected, starting at
   --  16#FFFF#, no final inversion.
   CRC_Initial : constant Unsigned_16 := 16#FFFF#;

   function Accumulate (CRC : Unsigned_16; B : Unsigned_8) return Unsigned_16;

   --  The checksum of Data, from CRC_Initial.
   function Checksum (Data : Byte_Array) return Unsigned_16;

   --  What this build knows about a message: its CRC_EXTRA and the length
   --  of its payload in full, extension fields included.
   type Message_Info is record
      Known      : Boolean := False;
      CRC_Extra  : Unsigned_8 := 0;
      Max_Length : Payload_Length := 0;
   end record;

   --  The frame fields besides the payload.
   type Header is record
      Sequence     : Unsigned_8 := 0;
      System       : Unsigned_8 := 0;
      Component    : Unsigned_8 := 0;
      Id           : Message_Id := 0;
      Compat_Flags : Unsigned_8 := 0;
   end record;

   --  A received message. Payload (1 .. Length) arrived; the bytes after
   --  it are zero, so a codec reads every field at its fixed offset
   --  whether or not the sender truncated it.
   type Message is record
      Head    : Header;
      Length  : Payload_Length := 0;
      Payload : Payload_Bytes := [others => 0];
   end record;
end MAVLink;
