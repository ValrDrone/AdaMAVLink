pragma SPARK_Mode (On);

--  Little-endian field access for the message codecs. Reads take a
--  received payload and the 1-based position of the field's first byte;
--  writes take any byte array and the position in it.

package MAVLink.Wire is
   function U8 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_8
   with Pre => At_Byte <= Max_Payload;
   function U16 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_16
   with Pre => At_Byte <= Max_Payload - 1;
   function U32 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_32
   with Pre => At_Byte <= Max_Payload - 3;
   function U64 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_64
   with Pre => At_Byte <= Max_Payload - 7;

   function I8 (P : Payload_Bytes; At_Byte : Positive) return Integer_8
   with Pre => At_Byte <= Max_Payload;
   function I16 (P : Payload_Bytes; At_Byte : Positive) return Integer_16
   with Pre => At_Byte <= Max_Payload - 1;
   function I32 (P : Payload_Bytes; At_Byte : Positive) return Integer_32
   with Pre => At_Byte <= Max_Payload - 3;
   function I64 (P : Payload_Bytes; At_Byte : Positive) return Integer_64
   with Pre => At_Byte <= Max_Payload - 7;

   function F32 (P : Payload_Bytes; At_Byte : Positive) return Float_Bits
   is (Float_Bits (U32 (P, At_Byte)))
   with Pre => At_Byte <= Max_Payload - 3;
   function F64 (P : Payload_Bytes; At_Byte : Positive) return Double_Bits
   is (Double_Bits (U64 (P, At_Byte)))
   with Pre => At_Byte <= Max_Payload - 7;

   --  B (At_Byte .. At_Byte + Size - 1) exists.
   function Fits (B : Byte_Array; At_Byte : Natural; Size : Positive)
                  return Boolean
   is (At_Byte >= B'First
       and then At_Byte <= B'Last
       and then B'Last - At_Byte >= Size - 1);

   procedure Put_U8 (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_8)
   with Pre => Fits (B, At_Byte, 1);
   procedure Put_U16
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_16)
   with Pre => Fits (B, At_Byte, 2);
   procedure Put_U32
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_32)
   with Pre => Fits (B, At_Byte, 4);
   procedure Put_U64
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_64)
   with Pre => Fits (B, At_Byte, 8);

   procedure Put_I8 (B : in out Byte_Array; At_Byte : Natural; V : Integer_8)
   with Pre => Fits (B, At_Byte, 1);
   procedure Put_I16 (B : in out Byte_Array; At_Byte : Natural; V : Integer_16)
   with Pre => Fits (B, At_Byte, 2);
   procedure Put_I32 (B : in out Byte_Array; At_Byte : Natural; V : Integer_32)
   with Pre => Fits (B, At_Byte, 4);
   procedure Put_I64 (B : in out Byte_Array; At_Byte : Natural; V : Integer_64)
   with Pre => Fits (B, At_Byte, 8);

   procedure Put_F32 (B : in out Byte_Array; At_Byte : Natural; V : Float_Bits)
   with Pre => Fits (B, At_Byte, 4);
   procedure Put_F64
     (B : in out Byte_Array; At_Byte : Natural; V : Double_Bits)
   with Pre => Fits (B, At_Byte, 8);
end MAVLink.Wire;
