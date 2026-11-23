package body Test_Mission_Handlers is
   procedure On_Mission_Ack (M : Message) is
      pragma Unreferenced (M);
   begin
      Seen := True;
   end On_Mission_Ack;
end Test_Mission_Handlers;
