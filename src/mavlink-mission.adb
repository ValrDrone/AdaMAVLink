pragma SPARK_Mode (On);

with MAVLink.Messages.Mission_Ack;
with MAVLink.Messages.Mission_Count;
with MAVLink.Messages.Mission_Request_Int;
with MAVLink.Messages.Mission_Request_List;

package body MAVLink.Mission is

   package Ack_Codec renames Messages.Mission_Ack;
   package Count_Codec renames Messages.Mission_Count;
   package Item_Codec renames Messages.Mission_Item_Int;
   package Request_Codec renames Messages.Mission_Request_Int;
   package List_Codec renames Messages.Mission_Request_List;

   Accepted     : constant Unsigned_8 := 0;  -- MAV_MISSION_ACCEPTED
   Mission_List : constant Unsigned_8 := 0;  -- MAV_MISSION_TYPE_MISSION

   --  True once at least Request_Timeout has passed since Sent_At. Valid
   --  under the package's monotonic-clock assumption: Now - Sent_At then
   --  holds the true elapsed time, unwrapped.
   function Timed_Out (Now, Sent_At : Milliseconds) return Boolean is
     (Now - Sent_At >= Request_Timeout);

   --  M was sent by the vehicle; a zero target field matches any sender.
   function From_Target
     (Target_System, Target_Component : Unsigned_8; M : Message)
      return Boolean
   is ((Target_System = 0 or else M.Head.System = Target_System)
       and then (Target_Component = 0
                 or else M.Head.Component = Target_Component));

   --  A received message's target fields address us; component zero is a
   --  broadcast to every component of our system.
   function For_Us
     (Own_System, Own_Component, Target_System, Target_Component : Unsigned_8)
      return Boolean
   is (Target_System = Own_System
       and then (Target_Component = Own_Component
                 or else Target_Component = 0));

   function Make_Message
     (Seq, System, Component : Unsigned_8; Id : Message_Id; P : Byte_Array)
      return Message
   is
      M : Message;
   begin
      M.Head := (Sequence => Seq, System => System, Component => Component,
                 Id => Id, Compat_Flags => 0);
      M.Length := P'Length;
      M.Payload (1 .. P'Length) := P;
      return M;
   end Make_Message;

   ------------
   -- Upload --
   ------------

   function Count_Message (U : Upload) return Message is
      C : constant Count_Codec.Content :=
        (Target_System    => U.Target_System,
         Target_Component => U.Target_Component,
         Count            => U.Count,
         Mission_Type     => Mission_List);
   begin
      return Make_Message (U.Sequence, U.Own_System, U.Own_Component,
                           Count_Codec.Id, Count_Codec.Pack (C));
   end Count_Message;

   --  Resend the last message: MISSION_COUNT until the vehicle has asked
   --  for an item, the last item after that. Gives up at Retry_Limit.
   procedure Resend (U : in out Upload; Now : Milliseconds; A : out Action) is
   begin
      A := (others => <>);
      if U.Attempts < Retry_Limit then
         U.Attempts := U.Attempts + 1;
         U.Sent_At := Now;
         A.Kind := Send_Message;
         if U.Next_Seq = 0 then
            A.Msg := Count_Message (U);
         else
            A.Msg := U.Last_Item;
            A.Msg.Head.Sequence := U.Sequence;
         end if;
         U.Sequence := U.Sequence + 1;
      else
         U.Phase := U_Timed_Out;
      end if;
   end Resend;

   procedure Start
     (U                : in out Upload;
      Now              : Milliseconds;
      Own_System       : Unsigned_8;
      Own_Component    : Unsigned_8;
      Target_System    : Unsigned_8;
      Target_Component : Unsigned_8;
      Count            : Unsigned_16;
      A                : out Action)
   is
   begin
      U.Phase := U_Waiting;
      U.Count := Count;
      U.Own_System := Own_System;
      U.Own_Component := Own_Component;
      U.Target_System := Target_System;
      U.Target_Component := Target_Component;
      U.Pending_Seq := 0;
      U.Next_Seq := 0;
      U.Result := Accepted;
      U.Attempts := 1;
      U.Sent_At := Now;
      U.Last_Now := Now;
      A := (others => <>);
      A.Kind := Send_Message;
      A.Msg := Count_Message (U);
      U.Sequence := U.Sequence + 1;
   end Start;

   procedure Process
     (U : in out Upload; Now : Milliseconds; M : Message; A : out Action)
   is
      Progress : Boolean := False;
   begin
      A := (others => <>);
      U.Last_Now := Now;
      if U.Phase = U_Waiting
        and then From_Target (U.Target_System, U.Target_Component, M)
      then
         if M.Head.Id = Ack_Codec.Id then
            declare
               C : constant Ack_Codec.Content := Ack_Codec.Unpack (M);
            begin
               if C.Mission_Type = Mission_List
                 and then For_Us (U.Own_System, U.Own_Component,
                                  C.Target_System, C.Target_Component)
               then
                  if C.Type_Field /= Accepted then
                     --  An error ends the transfer at any point.
                     U.Phase := U_Rejected;
                     U.Result := C.Type_Field;
                     Progress := True;
                  elsif U.Next_Seq = U.Count then
                     --  Accepted once every item has been sent; an early
                     --  acceptance is not one the protocol produces.
                     U.Phase := U_Done;
                     Progress := True;
                  end if;
               end if;
            end;
         elsif M.Head.Id = Request_Codec.Id then
            declare
               R : constant Request_Codec.Content := Request_Codec.Unpack (M);
            begin
               --  The next item, or a repeat of one already sent: the
               --  vehicle missed it, so answer again.
               if R.Mission_Type = Mission_List
                 and then For_Us (U.Own_System, U.Own_Component,
                                  R.Target_System, R.Target_Component)
                 and then R.Seq <= U.Next_Seq
                 and then R.Seq < U.Count
               then
                  U.Phase := U_Pending_Item;
                  U.Pending_Seq := R.Seq;
                  A.Kind := Want_Item;
                  A.Seq := R.Seq;
                  Progress := True;
               end if;
            end;
         end if;
      end if;

      if U.Phase = U_Waiting
        and then not Progress
        and then Timed_Out (Now, U.Sent_At)
      then
         Resend (U, Now, A);
      end if;
   end Process;

   procedure Tick (U : in out Upload; Now : Milliseconds; A : out Action) is
   begin
      A := (others => <>);
      U.Last_Now := Now;
      if U.Phase = U_Waiting and then Timed_Out (Now, U.Sent_At) then
         Resend (U, Now, A);
      end if;
   end Tick;

   procedure Send_Item
     (U   : in out Upload;
      Now : Milliseconds;
      C   : in out Messages.Mission_Item_Int.Content;
      A   : out Action)
   is
   begin
      U.Last_Now := Now;
      C.Seq := U.Pending_Seq;
      C.Target_System := U.Target_System;
      C.Target_Component := U.Target_Component;
      C.Mission_Type := Mission_List;
      U.Last_Item := Make_Message (U.Sequence, U.Own_System, U.Own_Component,
                                   Item_Codec.Id, Item_Codec.Pack (C));
      U.Sequence := U.Sequence + 1;
      A := (others => <>);
      A.Kind := Send_Message;
      A.Seq := U.Pending_Seq;
      A.Msg := U.Last_Item;
      if U.Pending_Seq = U.Next_Seq then
         --  A new item, not a repeat. Pending_Seq < Count, so no wrap.
         U.Next_Seq := U.Next_Seq + 1;
      end if;
      --  Wait for the next request, or the final MISSION_ACK.
      U.Phase := U_Waiting;
      U.Attempts := 1;
      U.Sent_At := Now;
   end Send_Item;

   --------------
   -- Download --
   --------------

   function List_Message (D : Download) return Message is
      C : constant List_Codec.Content :=
        (Target_System    => D.Target_System,
         Target_Component => D.Target_Component,
         Mission_Type     => Mission_List);
   begin
      return Make_Message (D.Sequence, D.Own_System, D.Own_Component,
                           List_Codec.Id, List_Codec.Pack (C));
   end List_Message;

   function Request_Message (D : Download) return Message is
      C : constant Request_Codec.Content :=
        (Target_System    => D.Target_System,
         Target_Component => D.Target_Component,
         Seq              => D.Expected_Seq,
         Mission_Type     => Mission_List);
   begin
      return Make_Message (D.Sequence, D.Own_System, D.Own_Component,
                           Request_Codec.Id, Request_Codec.Pack (C));
   end Request_Message;

   function Ack_Message (D : Download) return Message is
      C : constant Ack_Codec.Content :=
        (Target_System    => D.Target_System,
         Target_Component => D.Target_Component,
         Type_Field       => Accepted,
         Mission_Type     => Mission_List);
   begin
      return Make_Message (D.Sequence, D.Own_System, D.Own_Component,
                           Ack_Codec.Id, Ack_Codec.Pack (C));
   end Ack_Message;

   --  Resend MISSION_REQUEST_LIST or the current item request, or give
   --  up at Retry_Limit.
   procedure Resend (D : in out Download; Now : Milliseconds; A : out Action)
   is
   begin
      A := (others => <>);
      if D.Attempts < Retry_Limit then
         D.Attempts := D.Attempts + 1;
         D.Sent_At := Now;
         A.Kind := Send_Message;
         if D.Phase = D_Waiting_Count then
            A.Msg := List_Message (D);
         else
            A.Msg := Request_Message (D);
         end if;
         D.Sequence := D.Sequence + 1;
      else
         D.Phase := D_Timed_Out;
      end if;
   end Resend;

   procedure Start
     (D                : in out Download;
      Now              : Milliseconds;
      Own_System       : Unsigned_8;
      Own_Component    : Unsigned_8;
      Target_System    : Unsigned_8;
      Target_Component : Unsigned_8;
      A                : out Action)
   is
   begin
      D.Phase := D_Waiting_Count;
      D.Own_System := Own_System;
      D.Own_Component := Own_Component;
      D.Target_System := Target_System;
      D.Target_Component := Target_Component;
      D.Count := 0;
      D.Expected_Seq := 0;
      D.Result := Accepted;
      D.Attempts := 1;
      D.Sent_At := Now;
      D.Last_Now := Now;
      A := (others => <>);
      A.Kind := Send_Message;
      A.Msg := List_Message (D);
      D.Sequence := D.Sequence + 1;
   end Start;

   procedure Process
     (D : in out Download; Now : Milliseconds; M : Message; A : out Action)
   is
      Progress : Boolean := False;
   begin
      A := (others => <>);
      D.Last_Now := Now;
      if D.Phase in D_Waiting_Count | D_Waiting_Item
        and then From_Target (D.Target_System, D.Target_Component, M)
      then
         if M.Head.Id = Ack_Codec.Id then
            declare
               C : constant Ack_Codec.Content := Ack_Codec.Unpack (M);
            begin
               --  The vehicle refused the transfer (or cancelled it).
               if C.Mission_Type = Mission_List
                 and then For_Us (D.Own_System, D.Own_Component,
                                  C.Target_System, C.Target_Component)
                 and then C.Type_Field /= Accepted
               then
                  D.Phase := D_Rejected;
                  D.Result := C.Type_Field;
                  Progress := True;
               end if;
            end;
         elsif D.Phase = D_Waiting_Count
           and then M.Head.Id = Count_Codec.Id
         then
            declare
               N : constant Count_Codec.Content := Count_Codec.Unpack (M);
            begin
               if N.Mission_Type = Mission_List
                 and then For_Us (D.Own_System, D.Own_Component,
                                  N.Target_System, N.Target_Component)
               then
                  D.Count := N.Count;
                  D.Expected_Seq := 0;
                  D.Attempts := 1;
                  D.Sent_At := Now;
                  A.Kind := Send_Message;
                  if N.Count = 0 then
                     --  An empty mission: acknowledge and finish.
                     D.Phase := D_Done;
                     A.Msg := Ack_Message (D);
                  else
                     D.Phase := D_Waiting_Item;
                     A.Msg := Request_Message (D);
                  end if;
                  D.Sequence := D.Sequence + 1;
                  Progress := True;
               end if;
            end;
         elsif D.Phase = D_Waiting_Item
           and then M.Head.Id = Item_Codec.Id
         then
            declare
               I : constant Item_Codec.Content := Item_Codec.Unpack (M);
            begin
               if I.Mission_Type = Mission_List
                 and then For_Us (D.Own_System, D.Own_Component,
                                  I.Target_System, I.Target_Component)
                 and then I.Seq = D.Expected_Seq
               then
                  D.Phase := D_Pending_Item;
                  A.Kind := Got_Item;
                  A.Seq := I.Seq;
                  A.Msg := M;
                  Progress := True;
               end if;
            end;
         end if;
      end if;

      if D.Phase in D_Waiting_Count | D_Waiting_Item
        and then not Progress
        and then Timed_Out (Now, D.Sent_At)
      then
         Resend (D, Now, A);
      end if;
   end Process;

   procedure Tick (D : in out Download; Now : Milliseconds; A : out Action)
   is
   begin
      A := (others => <>);
      D.Last_Now := Now;
      if D.Phase in D_Waiting_Count | D_Waiting_Item
        and then Timed_Out (Now, D.Sent_At)
      then
         Resend (D, Now, A);
      end if;
   end Tick;

   procedure Item_Received
     (D : in out Download; Now : Milliseconds; A : out Action)
   is
   begin
      D.Last_Now := Now;
      --  Expected_Seq < Count here, so no wrap.
      D.Expected_Seq := D.Expected_Seq + 1;
      A := (others => <>);
      A.Kind := Send_Message;
      if D.Expected_Seq >= D.Count then
         D.Phase := D_Done;
         A.Msg := Ack_Message (D);
      else
         D.Phase := D_Waiting_Item;
         D.Attempts := 1;
         D.Sent_At := Now;
         A.Msg := Request_Message (D);
      end if;
      D.Sequence := D.Sequence + 1;
   end Item_Received;

   procedure Encode_Action
     (A     : Action;
      Frame : out Frames.Frame_Bytes;
      Last  : out Frames.Frame_Size)
   is
      L : constant Payload_Length :=
        Dialect.Info (A.Msg.Head.Id).Max_Length;
   begin
      Frames.Encode (A.Msg.Head, A.Msg.Payload (1 .. L), Frame, Last);
   end Encode_Action;

end MAVLink.Mission;
