pragma SPARK_Mode (On);

package body MAVLink is
   function Accumulate (CRC : Unsigned_16; B : Unsigned_8) return Unsigned_16
   is
      Low : Unsigned_8 := B xor Unsigned_8 (CRC and 16#FF#);
   begin
      Low := Low xor Shift_Left (Low, 4);
      return
        Shift_Right (CRC, 8)
        xor Shift_Left (Unsigned_16 (Low), 8)
        xor Shift_Left (Unsigned_16 (Low), 3)
        xor Shift_Right (Unsigned_16 (Low), 4);
   end Accumulate;

   function Checksum (Data : Byte_Array) return Unsigned_16 is
      CRC : Unsigned_16 := CRC_Initial;
   begin
      for B of Data loop
         CRC := Accumulate (CRC, B);
      end loop;
      return CRC;
   end Checksum;
end MAVLink;
