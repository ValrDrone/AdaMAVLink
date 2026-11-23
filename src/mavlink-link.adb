pragma SPARK_Mode (On);

package body MAVLink.Link is

   procedure Dispatch (T : Table; M : Message) is
   begin
      for E of T loop
         if E.Id = M.Head.Id and then E.H /= null then
            E.H (M);
            return;
         end if;
      end loop;
   end Dispatch;

   procedure Initialize (L : in out Link; System, Component : Unsigned_8) is
   begin
      L.System := System;
      L.Component := Component;
      L.Sequence := 0;
   end Initialize;

   procedure Send
     (L       : in out Link;
      Id      : Message_Id;
      Payload : Byte_Array;
      Frame   : out Frames.Frame_Bytes;
      Last    : out Frames.Frame_Size)
   is
      H : constant Header := (Sequence => L.Sequence,
                              System => L.System,
                              Component => L.Component,
                              Id => Id,
                              Compat_Flags => 0);
   begin
      L.Sequence := L.Sequence + 1;
      Frames.Encode (H, Payload, Frame, Last);
   end Send;

end MAVLink.Link;
