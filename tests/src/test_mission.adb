with Ada.Text_IO;
with Interfaces;                  use Interfaces;
with MAVLink;                     use MAVLink;
with MAVLink.Frames;
with MAVLink.Link;
with MAVLink.Mission;
with MAVLink.Messages.Mission_Ack;
with MAVLink.Messages.Mission_Count;
with MAVLink.Messages.Mission_Item_Int;
with MAVLink.Messages.Mission_Request_Int;
with MAVLink.Messages.Mission_Request_List;
with Test_Mission_Handlers;       use Test_Mission_Handlers;

--  The MAVLink.Mission state machines (upload and download) against a
--  scripted vehicle following https://mavlink.io/en/services/mission.html,
--  plus MAVLink.Link sequence numbering and dispatch.
procedure Test_Mission is
   package Mission renames MAVLink.Mission;
   package Link renames MAVLink.Link;
   use type Mission.Action_Kind, Mission.Status, Mission.Milliseconds;
   use type Frames.Event;
   package Mission_Ack renames MAVLink.Messages.Mission_Ack;
   package Mission_Count renames MAVLink.Messages.Mission_Count;
   package Mission_Item_Int renames MAVLink.Messages.Mission_Item_Int;
   package Mission_Request_Int renames MAVLink.Messages.Mission_Request_Int;
   package Mission_Request_List renames MAVLink.Messages.Mission_Request_List;

   Self_System    : constant := 255;  -- a GCS
   Self_Component : constant := 190;
   Peer_System    : constant := 1;    -- the vehicle's autopilot
   Peer_Component : constant := 1;
   Fence          : constant := 1;    -- MAV_MISSION_TYPE_FENCE
   No_Space       : constant := 4;    -- MAV_MISSION_NO_SPACE

   procedure Check (Condition : Boolean; Text : String) is
   begin
      if not Condition then
         raise Program_Error with Text;
      end if;
   end Check;

   function Msg
     (Id : Message_Id; P : Byte_Array; System : Unsigned_8 := Peer_System)
      return Message
   is
      M : Message;
   begin
      M.Head := (Sequence => 0, System => System, Component => Peer_Component,
                 Id => Id, Compat_Flags => 0);
      M.Length := P'Length;
      M.Payload (1 .. P'Length) := P;
      return M;
   end Msg;

   --  What the machine asked to send, through a real frame and parser.
   function Sent (A : Mission.Action; Text : String) return Message is
      F    : Frames.Frame_Bytes;
      Last : Frames.Frame_Size;
      P    : Frames.Parser;
      E    : Frames.Event;
   begin
      Check (A.Kind = Mission.Send_Message, Text & ": sends a message");
      Mission.Encode_Action (A, F, Last);
      for B of F (1 .. Last) loop
         Frames.Push (P, B, E);
      end loop;
      Check (E = Frames.Message_Ready, Text & ": frame parses");
      Check (Frames.Received (P).Head.System = Self_System
             and then Frames.Received (P).Head.Component = Self_Component,
             Text & ": sent with our identity");
      return Frames.Received (P);
   end Sent;

   function Ack_M
     (Type_Field   : Unsigned_8;
      Mission_Type : Unsigned_8 := 0) return Message
   is
     (Msg (Mission_Ack.Id,
           Mission_Ack.Pack
             ((Target_System => Self_System,
               Target_Component => Self_Component,
               Type_Field => Type_Field,
               Mission_Type => Mission_Type))));

   function Request_M
     (Seq          : Unsigned_16;
      Mission_Type : Unsigned_8 := 0;
      System       : Unsigned_8 := Peer_System) return Message
   is
     (Msg (Mission_Request_Int.Id,
           Mission_Request_Int.Pack
             ((Target_System => Self_System,
               Target_Component => Self_Component,
               Seq => Seq,
               Mission_Type => Mission_Type)),
           System));

   function Count_M (N : Unsigned_16; Mission_Type : Unsigned_8 := 0)
     return Message
   is
     (Msg (Mission_Count.Id,
           Mission_Count.Pack
             ((Target_System => Self_System,
               Target_Component => Self_Component,
               Count => N,
               Mission_Type => Mission_Type))));

   --  Autopilots set Autocontinue on every item; it does not end a list.
   function Item_M (Seq : Unsigned_16) return Message is
     (Msg (Mission_Item_Int.Id,
           Mission_Item_Int.Pack
             ((Target_System => Self_System,
               Target_Component => Self_Component,
               Seq => Seq, Autocontinue => 1, Command => 16,
               X => 473_977_000 + Integer_32 (Seq), others => <>))));

begin
   --  Upload: three items, with one repeated request, then accepted.
   declare
      U : Mission.Upload;
      A : Mission.Action;
      I : Mission_Item_Int.Content;
      M : Message;
      T : Mission.Milliseconds := 0;
   begin
      Check (Mission.State (U) = Mission.Idle, "upload starts idle");

      Mission.Start (U, T, Self_System, Self_Component,
                     Peer_System, Peer_Component, 3, A);
      M := Sent (A, "upload start");
      Check (M.Head.Id = Mission_Count.Id, "upload starts with the count");
      Check (Mission_Count.Unpack (M).Count = 3
             and then Mission_Count.Unpack (M).Target_System = Peer_System
             and then Mission_Count.Unpack (M).Mission_Type = 0,
             "count carries three mission items for the vehicle");

      for S in Unsigned_16 range 0 .. 2 loop
         T := T + 100;
         Mission.Process (U, T, Request_M (S), A);
         Check (A.Kind = Mission.Want_Item and then A.Seq = S,
                "request asks for the item");
         Check (Mission.Item_Pending (U), "answer pending");

         --  While the answer is pending the machine ignores messages.
         T := T + 10;
         Mission.Process (U, T, Ack_M (0), A);
         Check (A.Kind = Mission.None_A, "frozen while an item is pending");

         T := T + 10;
         I := (Command => 16, X => 100 + Integer_32 (S), others => <>);
         Mission.Send_Item (U, T, I, A);
         M := Sent (A, "item");
         Check (M.Head.Id = Mission_Item_Int.Id
                and then Mission_Item_Int.Unpack (M).Seq = S
                and then Mission_Item_Int.Unpack (M).X = 100 + Integer_32 (S)
                and then Mission_Item_Int.Unpack (M).Target_System
                           = Peer_System,
                "item stamped with sequence and target");

         if S = 1 then
            --  The vehicle missed item 1 and asks again.
            T := T + 100;
            Mission.Process (U, T, Request_M (1), A);
            Check (A.Kind = Mission.Want_Item and then A.Seq = 1,
                   "repeated request is answered");
            Mission.Send_Item (U, T, I, A);
            Check (Mission_Item_Int.Unpack (Sent (A, "repeat")).Seq = 1,
                   "repeat carries the same sequence");
         end if;
      end loop;

      Check (Mission.State (U) = Mission.In_Progress,
             "upload waits for the final ack");
      T := T + 100;
      Mission.Process (U, T, Ack_M (0), A);
      Check (Mission.State (U) = Mission.Succeeded
             and then Mission.Result (U) = 0, "accepted ack ends the upload");
   end;

   --  Upload: messages not for this transfer are ignored.
   declare
      U : Mission.Upload;
      A : Mission.Action;
   begin
      Mission.Start (U, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, 2, A);
      Mission.Process (U, 10, Request_M (0, System => 9), A);
      Check (A.Kind = Mission.None_A, "request from another system ignored");
      Mission.Process (U, 20, Request_M (0, Mission_Type => Fence), A);
      Check (A.Kind = Mission.None_A, "fence request ignored");
      Mission.Process (U, 30, Request_M (1), A);
      Check (A.Kind = Mission.None_A, "request skipping ahead ignored");
      Mission.Process (U, 40, Ack_M (0), A);
      Check (Mission.State (U) = Mission.In_Progress,
             "acceptance before the items ignored");
   end;

   --  Upload: an empty mission (clears the vehicle's list).
   declare
      U : Mission.Upload;
      A : Mission.Action;
   begin
      Mission.Start (U, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, 0, A);
      Mission.Process (U, 50, Ack_M (0), A);
      Check (Mission.State (U) = Mission.Succeeded, "empty upload accepted");
   end;

   --  Upload: the vehicle rejects it.
   declare
      U : Mission.Upload;
      A : Mission.Action;
   begin
      Mission.Start (U, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, 500, A);
      Mission.Process (U, 50, Ack_M (No_Space), A);
      Check (Mission.State (U) = Mission.Rejected
             and then Mission.Result (U) = No_Space,
             "error ack rejects with its result");
   end;

   --  Upload: a silent vehicle gets the count again, then the upload
   --  times out.
   declare
      U : Mission.Upload;
      A : Mission.Action;
   begin
      Mission.Start (U, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, 1, A);
      Mission.Tick (U, 2_999, A);
      Check (A.Kind = Mission.None_A, "no retry before the timeout");
      for K in 1 .. Mission.Retry_Limit - 1 loop
         Mission.Tick (U, Mission.Milliseconds (K * 3_000), A);
         Check (Sent (A, "count retry").Head.Id = Mission_Count.Id,
                "timeout resends the count");
      end loop;
      Mission.Tick (U, 20_000, A);
      Check (A.Kind = Mission.None_A
             and then Mission.State (U) = Mission.Timed_Out,
             "gives up after the limit");
   end;

   --  Upload: after an item, a silent vehicle gets that item again.
   declare
      U : Mission.Upload;
      A : Mission.Action;
      I : Mission_Item_Int.Content := (Command => 22, others => <>);
   begin
      Mission.Start (U, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, 2, A);
      Mission.Process (U, 100, Request_M (0), A);
      Mission.Send_Item (U, 110, I, A);
      Mission.Process (U, 3_200, Ack_M (0, Mission_Type => Fence), A);
      Check (Sent (A, "item retry").Head.Id = Mission_Item_Int.Id
             and then Mission_Item_Int.Unpack (Sent (A, "item retry")).Command
                        = 22,
             "timeout resends the last item");
   end;

   --  Download: three items, then our acceptance.
   declare
      D : Mission.Download;
      A : Mission.Action;
      M : Message;
      T : Mission.Milliseconds := 0;
   begin
      Check (Mission.State (D) = Mission.Idle, "download starts idle");

      Mission.Start (D, T, Self_System, Self_Component,
                     Peer_System, Peer_Component, A);
      M := Sent (A, "download start");
      Check (M.Head.Id = Mission_Request_List.Id
             and then Mission_Request_List.Unpack (M).Target_System
                        = Peer_System,
             "download starts with the list request");

      T := T + 100;
      Mission.Process (D, T, Count_M (3), A);
      Check (Mission.Count (D) = 3, "count recorded");
      M := Sent (A, "first request");
      Check (M.Head.Id = Mission_Request_Int.Id
             and then Mission_Request_Int.Unpack (M).Seq = 0,
             "count answered with a request for item zero");

      for S in Unsigned_16 range 0 .. 2 loop
         T := T + 100;
         Mission.Process (D, T, Item_M (S + 1), A);
         Check (A.Kind = Mission.None_A, "item out of order ignored");

         T := T + 100;
         Mission.Process (D, T, Item_M (S), A);
         Check (A.Kind = Mission.Got_Item and then A.Seq = S
                and then Mission_Item_Int.Unpack (A.Msg).X
                           = 473_977_000 + Integer_32 (S),
                "item delivered");
         Check (Mission.State (D) = Mission.In_Progress,
                "autocontinue does not end the download");

         T := T + 50;
         Mission.Item_Received (D, T, A);
         M := Sent (A, "after item");
         if S < 2 then
            Check (M.Head.Id = Mission_Request_Int.Id
                   and then Mission_Request_Int.Unpack (M).Seq = S + 1,
                   "next item requested");
         else
            Check (M.Head.Id = Mission_Ack.Id
                   and then Mission_Ack.Unpack (M).Type_Field = 0
                   and then Mission_Ack.Unpack (M).Target_System
                              = Peer_System,
                   "last item acknowledged");
         end if;
      end loop;

      Check (Mission.State (D) = Mission.Succeeded, "download done");
      Mission.Process (D, T + 100, Item_M (2), A);
      Check (A.Kind = Mission.None_A, "late repeat after the end ignored");
   end;

   --  Download: an empty mission is acknowledged at once.
   declare
      D : Mission.Download;
      A : Mission.Action;
   begin
      Mission.Start (D, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, A);
      Mission.Process (D, 10, Count_M (2, Mission_Type => Fence), A);
      Check (A.Kind = Mission.None_A, "fence count ignored");
      Mission.Process (D, 20, Count_M (0), A);
      Check (Sent (A, "empty").Head.Id = Mission_Ack.Id
             and then Mission.State (D) = Mission.Succeeded,
             "empty mission acknowledged");
   end;

   --  Download: a silent vehicle gets the list request again, then the
   --  download times out.
   declare
      D : Mission.Download;
      A : Mission.Action;
   begin
      Mission.Start (D, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, A);
      for K in 1 .. Mission.Retry_Limit - 1 loop
         Mission.Tick (D, Mission.Milliseconds (K * 3_000), A);
         Check (Sent (A, "list retry").Head.Id = Mission_Request_List.Id,
                "timeout resends the list request");
      end loop;
      Mission.Tick (D, 20_000, A);
      Check (Mission.State (D) = Mission.Timed_Out,
             "gives up after the limit");
   end;

   --  Download: a lost item is requested again.
   declare
      D : Mission.Download;
      A : Mission.Action;
      M : Message;
   begin
      Mission.Start (D, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, A);
      Mission.Process (D, 10, Count_M (2), A);
      Mission.Process (D, 20, Item_M (0), A);
      Mission.Item_Received (D, 30, A);
      Mission.Tick (D, 3_030, A);
      M := Sent (A, "request retry");
      Check (M.Head.Id = Mission_Request_Int.Id
             and then Mission_Request_Int.Unpack (M).Seq = 1,
             "timeout resends the current request");
   end;

   --  Download: the vehicle refuses.
   declare
      D : Mission.Download;
      A : Mission.Action;
   begin
      Mission.Start (D, 0, Self_System, Self_Component,
                     Peer_System, Peer_Component, A);
      Mission.Process (D, 10, Ack_M (No_Space), A);
      Check (Mission.State (D) = Mission.Rejected
             and then Mission.Result (D) = No_Space,
             "error ack rejects the download");
   end;

   --  Link: identity stamping and sequence numbering.
   declare
      L    : Link.Link;
      P    : constant Mission_Ack.Payload :=
        Mission_Ack.Pack ((Target_System => Peer_System,
                           Target_Component => Peer_Component,
                           Type_Field => 0, others => <>));
      F    : Frames.Frame_Bytes;
      Last : Frames.Frame_Size;
      R    : Frames.Parser;
      E    : Frames.Event;
   begin
      Link.Initialize (L, Self_System, Self_Component);
      Link.Send (L, Mission_Ack.Id, P, F, Last);
      for B of F (1 .. Last) loop
         Frames.Push (R, B, E);
      end loop;
      Check (Frames.Received (R).Head.System = Self_System
             and then Frames.Received (R).Head.Component = Self_Component,
             "link stamps its identity");
      Check (Frames.Received (R).Head.Sequence = 0, "first sequence is zero");
      Link.Send (L, Mission_Ack.Id, P, F, Last);
      for B of F (1 .. Last) loop
         Frames.Push (R, B, E);
      end loop;
      Check (Frames.Received (R).Head.Sequence = 1, "sequence advances");
   end;

   --  Link: dispatch to a registered handler, silence for the rest.
   declare
      H : constant Link.Handler := On_Mission_Ack'Access;
      T : constant Link.Table := [(Mission_Ack.Id, H)];
   begin
      Seen := False;
      Link.Dispatch (T, Ack_M (0));
      Check (Seen, "dispatch calls the handler");

      Seen := False;
      Link.Dispatch (T, Request_M (0));
      Check (not Seen, "unhandled id is silent");

      declare
         T2 : constant Link.Table := [(Mission_Ack.Id, null)];
      begin
         Seen := False;
         Link.Dispatch (T2, Ack_M (0));
         Check (not Seen, "null handler is silent");
      end;
   end;

   Ada.Text_IO.Put_Line ("MISSION and LINK tests passed");
end Test_Mission;
