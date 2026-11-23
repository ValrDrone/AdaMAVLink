pragma SPARK_Mode (On);

--  IEEE 754 float and double fields: whether the bits hold a finite value,
--  and the conversions between finite values and bits. MAVLink sends NaN
--  for "unknown" in many fields; such a field is not finite, and a reader
--  checks that before converting.

package MAVLink.Floats is
   pragma Compile_Time_Error
     (Float'Size /= 32 or else Float'Machine_Mantissa /= 24,
      "Float must be IEEE 754 single precision");
   pragma Compile_Time_Error
     (Long_Float'Size /= 64 or else Long_Float'Machine_Mantissa /= 53,
      "Long_Float must be IEEE 754 double precision");

   --  The quiet NaN MAVLink senders use for "unknown".
   NaN        : constant Float_Bits := 16#7FC0_0000#;
   Double_NaN : constant Double_Bits := 16#7FF8_0000_0000_0000#;

   function Is_Finite (B : Float_Bits) return Boolean
   is ((B and 16#7F80_0000#) /= 16#7F80_0000#);

   function Is_Finite (B : Double_Bits) return Boolean
   is ((B and 16#7FF0_0000_0000_0000#) /= 16#7FF0_0000_0000_0000#);

   function To_Float (B : Float_Bits) return Float
   with Pre => Is_Finite (B);

   function To_Long_Float (B : Double_Bits) return Long_Float
   with Pre => Is_Finite (B);

   --  The IEEE 754 bits of X, whatever they are: for a non-finite X the
   --  result is a NaN or infinity pattern; check it with Is_Finite before
   --  converting back.
   function To_Bits (X : Float) return Float_Bits;

   function To_Bits (X : Long_Float) return Double_Bits;
end MAVLink.Floats;
