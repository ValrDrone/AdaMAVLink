pragma SPARK_Mode (On);

package body MAVLink.Wire is
   function U8 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_8
   is (P (At_Byte));

   function U16 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_16
   is (Unsigned_16 (P (At_Byte))
       or Shift_Left (Unsigned_16 (P (At_Byte + 1)), 8));

   function U32 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_32
   is (Unsigned_32 (P (At_Byte))
       or Shift_Left (Unsigned_32 (P (At_Byte + 1)), 8)
       or Shift_Left (Unsigned_32 (P (At_Byte + 2)), 16)
       or Shift_Left (Unsigned_32 (P (At_Byte + 3)), 24));

   function U64 (P : Payload_Bytes; At_Byte : Positive) return Unsigned_64
   is (Unsigned_64 (U32 (P, At_Byte))
       or Shift_Left (Unsigned_64 (U32 (P, At_Byte + 4)), 32));

   function I8 (P : Payload_Bytes; At_Byte : Positive) return Integer_8 is
      X : constant Unsigned_8 := P (At_Byte);
   begin
      return
        (if X >= 2**7 then Integer_8 (Integer (X) - 2**8)
         else Integer_8 (X));
   end I8;

   function I16 (P : Payload_Bytes; At_Byte : Positive) return Integer_16 is
      X : constant Unsigned_16 := U16 (P, At_Byte);
   begin
      return
        (if X >= 2**15 then Integer_16 (Integer_32 (X) - 2**16)
         else Integer_16 (X));
   end I16;

   function I32 (P : Payload_Bytes; At_Byte : Positive) return Integer_32 is
      X : constant Unsigned_32 := U32 (P, At_Byte);
   begin
      return
        (if X >= 2**31 then -Integer_32 (not X) - 1
         else Integer_32 (X));
   end I32;

   function I64 (P : Payload_Bytes; At_Byte : Positive) return Integer_64 is
      X : constant Unsigned_64 := U64 (P, At_Byte);
   begin
      return
        (if X >= 2**63 then -Integer_64 (not X) - 1
         else Integer_64 (X));
   end I64;

   procedure Put_U8 (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_8)
   is
   begin
      B (At_Byte) := V;
   end Put_U8;

   procedure Put_U16
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_16) is
   begin
      B (At_Byte) := Unsigned_8 (V and 16#FF#);
      B (At_Byte + 1) := Unsigned_8 (Shift_Right (V, 8));
   end Put_U16;

   procedure Put_U32
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_32) is
   begin
      for K in 0 .. 3 loop
         B (At_Byte + K) := Unsigned_8 (Shift_Right (V, 8 * K) and 16#FF#);
      end loop;
   end Put_U32;

   procedure Put_U64
     (B : in out Byte_Array; At_Byte : Natural; V : Unsigned_64) is
   begin
      for K in 0 .. 7 loop
         B (At_Byte + K) := Unsigned_8 (Shift_Right (V, 8 * K) and 16#FF#);
      end loop;
   end Put_U64;

   procedure Put_I8 (B : in out Byte_Array; At_Byte : Natural; V : Integer_8)
   is
   begin
      Put_U8 (B, At_Byte, Unsigned_8'Mod (V));
   end Put_I8;

   procedure Put_I16 (B : in out Byte_Array; At_Byte : Natural; V : Integer_16)
   is
   begin
      Put_U16 (B, At_Byte, Unsigned_16'Mod (V));
   end Put_I16;

   procedure Put_I32 (B : in out Byte_Array; At_Byte : Natural; V : Integer_32)
   is
   begin
      Put_U32 (B, At_Byte, Unsigned_32'Mod (V));
   end Put_I32;

   procedure Put_I64 (B : in out Byte_Array; At_Byte : Natural; V : Integer_64)
   is
   begin
      Put_U64 (B, At_Byte, Unsigned_64'Mod (V));
   end Put_I64;

   procedure Put_F32 (B : in out Byte_Array; At_Byte : Natural; V : Float_Bits)
   is
   begin
      Put_U32 (B, At_Byte, Unsigned_32 (V));
   end Put_F32;

   procedure Put_F64
     (B : in out Byte_Array; At_Byte : Natural; V : Double_Bits) is
   begin
      Put_U64 (B, At_Byte, Unsigned_64 (V));
   end Put_F64;
end MAVLink.Wire;
