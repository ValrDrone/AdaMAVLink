pragma SPARK_Mode (On);

package body MAVLink.Frames is
   procedure Finish (P : in out Parser; Result : out Event) is
   begin
      Result := P.Pending;
      case P.Pending is
         when Message_Ready     =>
            P.Stats.Messages := P.Stats.Messages + 1;

         when Bad_Checksum      =>
            P.Stats.Bad_Checksums := P.Stats.Bad_Checksums + 1;

         when Unknown_Message   =>
            P.Stats.Unknown_Messages := P.Stats.Unknown_Messages + 1;

         when Unsupported_Flags =>
            P.Stats.Unsupported_Flags := P.Stats.Unsupported_Flags + 1;

         when None              =>
            null;
      end case;
      P.Pending := None;
      P.Phase := Start;
   end Finish;

   --  The frame's outcome once its checksum's high byte has arrived.
   function Outcome (P : Parser; High : Unsigned_8) return Event is
      Info     : constant Message_Info := Dialect.Info (P.Current.Head.Id);
      Received : constant Unsigned_16 :=
        Unsigned_16 (P.CRC_Low) or Shift_Left (Unsigned_16 (High), 8);
   begin
      if P.Incompat /= 0 then
         return Unsupported_Flags;
      elsif not Info.Known then
         return Unknown_Message;
      elsif Accumulate (P.CRC, Info.CRC_Extra) /= Received then
         return Bad_Checksum;
      else
         return Message_Ready;
      end if;
   end Outcome;

   procedure Push (P : in out Parser; B : Unsigned_8; Result : out Event) is
   begin
      Result := None;
      if P.Phase in Length .. Payload then
         P.CRC := Accumulate (P.CRC, B);
      end if;

      case P.Phase is
         when Start         =>
            if B = Start_V2 then
               P.Current := (others => <>);
               P.Count := 0;
               P.Incompat := 0;
               P.CRC := CRC_Initial;
               P.Pending := None;
               P.Phase := Length;
            else
               P.Stats.Skipped_Bytes := P.Stats.Skipped_Bytes + 1;
            end if;

         when Length        =>
            P.Current.Length := Payload_Length (B);
            P.Phase := Incompat;

         when Incompat      =>
            P.Incompat := B;
            P.Phase := Compat;

         when Compat        =>
            P.Current.Head.Compat_Flags := B;
            P.Phase := Sequence;

         when Sequence      =>
            P.Current.Head.Sequence := B;
            P.Phase := System;

         when System        =>
            P.Current.Head.System := B;
            P.Phase := Component;

         when Component     =>
            P.Current.Head.Component := B;
            P.Phase := Id_Low;

         when Id_Low        =>
            P.Current.Head.Id := Message_Id (B);
            P.Phase := Id_Middle;

         when Id_Middle     =>
            P.Current.Head.Id :=
              P.Current.Head.Id mod 2**8 + Message_Id (B) * 2**8;
            P.Phase := Id_High;

         when Id_High       =>
            P.Current.Head.Id :=
              P.Current.Head.Id mod 2**16 + Message_Id (B) * 2**16;
            P.Phase :=
              (if P.Current.Length = 0 then Checksum_Low else Payload);

         when Payload       =>
            if P.Count < P.Current.Length then
               P.Count := P.Count + 1;
               P.Current.Payload (P.Count) := B;
               if P.Count = P.Current.Length then
                  P.Phase := Checksum_Low;
               end if;
            else
               P.Phase := Start;
            end if;

         when Checksum_Low  =>
            P.CRC_Low := B;
            P.Phase := Checksum_High;

         when Checksum_High =>
            P.Pending := Outcome (P, B);
            if (P.Incompat and Incompat_Signed) /= 0 then
               P.Remaining := Signature_Length;
               P.Phase := Signature;
            else
               Finish (P, Result);
            end if;

         when Signature     =>
            if P.Remaining > 1 then
               P.Remaining := P.Remaining - 1;
            else
               P.Remaining := 0;
               Finish (P, Result);
            end if;
      end case;
   end Push;

   procedure Encode
     (Head    : Header;
      Payload : Byte_Array;
      Frame   : out Frame_Bytes;
      Last    : out Frame_Size)
   is
      Info : constant Message_Info := Dialect.Info (Head.Id);
      N    : Positive range 1 .. Max_Payload := Payload'Last;
      CRC  : Unsigned_16;
   begin
      while N > 1 and then Payload (N) = 0 loop
         pragma Loop_Invariant (N in Payload'Range);
         N := N - 1;
      end loop;

      Frame := [others => 0];
      Frame (1) := Start_V2;
      Frame (2) := Unsigned_8 (N);
      Frame (3) := 0;
      Frame (4) := Head.Compat_Flags;
      Frame (5) := Head.Sequence;
      Frame (6) := Head.System;
      Frame (7) := Head.Component;
      Frame (8) := Unsigned_8 (Head.Id mod 2**8);
      Frame (9) := Unsigned_8 (Head.Id / 2**8 mod 2**8);
      Frame (10) := Unsigned_8 (Head.Id / 2**16);
      Frame (Header_Length + 1 .. Header_Length + N) := Payload (1 .. N);

      CRC :=
        Accumulate (Checksum (Frame (2 .. Header_Length + N)), Info.CRC_Extra);
      Frame (Header_Length + N + 1) := Unsigned_8 (CRC and 16#FF#);
      Frame (Header_Length + N + 2) := Unsigned_8 (Shift_Right (CRC, 8));
      Last := Header_Length + N + Checksum_Length;
   end Encode;
end MAVLink.Frames;
