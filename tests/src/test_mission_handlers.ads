with MAVLink; use MAVLink;

--  A dispatch handler for Test_Mission. MAVLink.Link.Handler is a
--  library-level access type, so the handler must live at library level.
package Test_Mission_Handlers is
   Seen : Boolean := False;

   procedure On_Mission_Ack (M : Message);
end Test_Mission_Handlers;
