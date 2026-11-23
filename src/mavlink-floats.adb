with Ada.Unchecked_Conversion;

--  Outside SPARK: the provers do not model a float's bits, so these four
--  conversions are trusted, not proved. Their contracts hold because a
--  finite bit pattern is a valid float of the same size (compile-time
--  checks in the spec), and any float converts to its exact IEEE 754 bits.
package body MAVLink.Floats
  with SPARK_Mode => Off
is
   function From_32 is new Ada.Unchecked_Conversion (Float_Bits, Float);
   function From_64 is new Ada.Unchecked_Conversion (Double_Bits, Long_Float);
   function Into_32 is new Ada.Unchecked_Conversion (Float, Float_Bits);
   function Into_64 is new Ada.Unchecked_Conversion (Long_Float, Double_Bits);

   function To_Float (B : Float_Bits) return Float is (From_32 (B));

   function To_Long_Float (B : Double_Bits) return Long_Float
   is (From_64 (B));

   function To_Bits (X : Float) return Float_Bits is (Into_32 (X));

   function To_Bits (X : Long_Float) return Double_Bits is (Into_64 (X));
end MAVLink.Floats;
