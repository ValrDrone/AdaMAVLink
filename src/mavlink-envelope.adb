pragma SPARK_Mode (On);

package body MAVLink.Envelope is

   -----------
   -- Split --
   -----------

   procedure Split
     (Frame    : Byte_Array;
      Index    : Natural;
      Envelope : out Envelope_Bytes;
      Last     : out Envelope_Size)
   is
      Total : constant Chunk_Count := Chunks (Frame'Length);
      First : constant Natural := Index * Max_Data;
      Size  : constant Positive range 1 .. Max_Data :=
        (if Index + 1 < Total then Max_Data else Frame'Length - First);
   begin
      pragma Assert (First + Size <= Frame'Length);
      Envelope := [others => 0];
      Envelope (1) := Unsigned_8 ((Total - 1) * 16 + Index);
      Envelope (2) := Unsigned_8 (Size);
      for I in 1 .. Size loop
         Envelope (2 + I) := Frame (Frame'First + (First + I - 1));
      end loop;
      Last := 2 + Size;
   end Split;

   ----------
   -- Join --
   ----------

   procedure Join (J : in out Joiner; Envelope : Byte_Array; Result : out Event)
   is
      procedure Drop (Why : Event) with
        Post => not J.Active and then Result = Why;

      procedure Drop (Why : Event) is
      begin
         if J.Active then
            J.Stats.Abandoned := J.Stats.Abandoned + 1;
         end if;
         J.Stats.Dropped := J.Stats.Dropped + 1;
         J.Active := False;
         J.Length := 0;
         Result := Why;
      end Drop;

      Header : Unsigned_8;
      Size   : Natural;
      Final  : Index_Value;
      Index  : Index_Value;
   begin
      J.Ready := 0;
      Result := None;

      if Envelope'Length < 3 then
         Drop (Malformed);
         return;
      end if;

      Header := Envelope (Envelope'First);
      Size := Natural (Envelope (Envelope'First + 1));
      Final := Index_Value (Header / 16);
      Index := Index_Value (Header mod 16);

      if Size = 0
        or else Size > Max_Data
        or else Envelope'Length < 2 + Size
        or else Index > Final
      then
         Drop (Malformed);
         return;
      end if;

      if Index = 0 then
         if J.Active then
            J.Stats.Abandoned := J.Stats.Abandoned + 1;
         end if;
         J.Active := True;
         J.Final := Final;
         J.Length := 0;
      elsif not J.Active or else Index /= J.Next or else Final /= J.Final then
         Drop (Out_Of_Order);
         return;
      end if;

      if Size > Max_Frame - J.Length then
         Drop (Malformed);
         return;
      end if;

      for I in 1 .. Size loop
         pragma Loop_Invariant (J.Length + Size <= Max_Frame);
         J.Buffer (J.Length + I) := Envelope (Envelope'First + 1 + I);
      end loop;
      J.Length := J.Length + Size;

      if Index = Final then
         J.Active := False;
         J.Ready := J.Length;
         J.Length := 0;
         J.Stats.Frames := J.Stats.Frames + 1;
         Result := Frame_Ready;
      else
         J.Next := Index + 1;
      end if;
   end Join;

   -----------
   -- Reset --
   -----------

   procedure Reset (J : out Joiner) is
   begin
      J := (others => <>);
   end Reset;

end MAVLink.Envelope;
