pragma SPARK_Mode (On);

--  The MAVLink mission protocol (https://mavlink.io/en/services/mission.html)
--  from the initiating side, as two state machines: uploading a mission to
--  a vehicle (Upload) and downloading one from it (Download). Only the
--  MAV_MISSION_TYPE_MISSION list is transferred; messages for other mission
--  types (fence, rally), from other systems or addressed to others are
--  ignored.
--
--  Upload:   -> MISSION_COUNT
--            <- MISSION_REQUEST_INT (seq)    -> MISSION_ITEM_INT (seq)
--            ...                             (repeated requests are answered)
--            <- MISSION_ACK                  accepted ends it; an error
--                                            result rejects it at any point
--
--  Download: -> MISSION_REQUEST_LIST
--            <- MISSION_COUNT (n)
--            -> MISSION_REQUEST_INT (seq)    <- MISSION_ITEM_INT (seq)
--            ...                             for seq in 0 .. n - 1
--            -> MISSION_ACK (accepted)
--
--  The machines own the handshake and retries: when the vehicle does not
--  answer within Request_Timeout they resend their last message, up to
--  Retry_Limit sends per step. The caller owns the transport and the
--  mission data. Every entry point returns an Action:
--
--    Send_Message  A.Msg is a fully packed message; send it with
--                  Encode_Action.
--    Want_Item     (upload) the vehicle asked for item A.Seq; look it up
--                  and answer with Send_Item.
--    Got_Item      (download) A.Msg holds item A.Seq; store it, then
--                  call Item_Received.
--
--  While a machine waits for one of those answers (Item_Pending), Process
--  and Tick ignore messages and return None_A.
--
--  Time is caller-supplied in milliseconds and must be monotonic across
--  calls to the same machine; assume less than 2**31 ms elapse between
--  Start and a terminal status, so timeout arithmetic does not wrap.

with MAVLink.Dialect;
with MAVLink.Frames;
with MAVLink.Messages.Mission_Item_Int;

package MAVLink.Mission is
   type Milliseconds is new Unsigned_32;

   --  How long to wait for the vehicle's next message before resending.
   Request_Timeout : constant Milliseconds := 3_000;
   --  Sends per step before the machine gives up (first send plus
   --  retries).
   Retry_Limit : constant := 4;

   type Action_Kind is (None_A, Send_Message, Want_Item, Got_Item);

   type Action is record
      Kind : Action_Kind := None_A;
      Seq  : Unsigned_16 := 0;  -- Want_Item and Got_Item: the item
      Msg  : Message;           -- Send_Message and Got_Item
   end record;

   --  Encode A.Msg into a v2 frame. The machines only produce messages
   --  they packed themselves, so the preconditions hold for every
   --  Send_Message action they return.
   procedure Encode_Action
     (A     : Action;
      Frame : out Frames.Frame_Bytes;
      Last  : out Frames.Frame_Size)
   with Pre => A.Kind = Send_Message
              and then Dialect.Is_Known (A.Msg.Head.Id)
              and then A.Msg.Length =
                         Dialect.Info (A.Msg.Head.Id).Max_Length;

   --  Rejected: the vehicle answered with an error MISSION_ACK (see
   --  Result). Timed_Out: it stopped answering.
   type Status is (Idle, In_Progress, Succeeded, Rejected, Timed_Out);

   ------------
   -- Upload --
   ------------

   type Upload is private;

   --  The time of the last call on this machine; Now arguments must be
   --  at or after it.
   function Last_Time (U : Upload) return Milliseconds;
   function State (U : Upload) return Status;
   function Item_Pending (U : Upload) return Boolean;
   --  The MAV_MISSION_RESULT (MAVLink.Enums) of the vehicle's error
   --  MISSION_ACK; 0 unless State is Rejected.
   function Result (U : Upload) return Unsigned_8;

   --  Begin uploading Count items (zero clears the vehicle's mission).
   --  Restarts a machine in any state.
   procedure Start
     (U                : in out Upload;
      Now              : Milliseconds;
      Own_System       : Unsigned_8;
      Own_Component    : Unsigned_8;
      Target_System    : Unsigned_8;
      Target_Component : Unsigned_8;
      Count            : Unsigned_16;
      A                : out Action)
   with Pre  => Now >= Last_Time (U),
        Post => A.Kind = Send_Message
                and then State (U) = In_Progress
                and then not Item_Pending (U)
                and then Last_Time (U) = Now;

   --  Feed one received message (any ID) and check for a timeout.
   procedure Process
     (U : in out Upload; Now : Milliseconds; M : Message; A : out Action)
   with Pre  => Now >= Last_Time (U),
        Post => Last_Time (U) = Now
                and then A.Kind in None_A | Send_Message | Want_Item
                and then (if A.Kind = Want_Item then Item_Pending (U));

   --  Check for a timeout without a new message.
   procedure Tick (U : in out Upload; Now : Milliseconds; A : out Action)
   with Pre  => Now >= Last_Time (U),
        Post => Last_Time (U) = Now
                and then A.Kind in None_A | Send_Message;

   --  Answer the vehicle's last request with C. The machine fills the
   --  sequence number, target and mission type fields; everything else
   --  in C is sent as given. A is the packed MISSION_ITEM_INT.
   procedure Send_Item
     (U   : in out Upload;
      Now : Milliseconds;
      C   : in out Messages.Mission_Item_Int.Content;
      A   : out Action)
   with Pre  => Item_Pending (U) and then Now >= Last_Time (U),
        Post => A.Kind = Send_Message
                and then not Item_Pending (U)
                and then Last_Time (U) = Now;

   --------------
   -- Download --
   --------------

   type Download is private;

   --  The time of the last call on this machine; Now arguments must be
   --  at or after it.
   function Last_Time (D : Download) return Milliseconds;
   function State (D : Download) return Status;
   function Item_Pending (D : Download) return Boolean;
   --  The number of items the vehicle announced; 0 until it has.
   function Count (D : Download) return Unsigned_16;
   --  The MAV_MISSION_RESULT of the vehicle's error MISSION_ACK; 0 unless
   --  State is Rejected.
   function Result (D : Download) return Unsigned_8;

   --  Begin downloading. Restarts a machine in any state.
   procedure Start
     (D                : in out Download;
      Now              : Milliseconds;
      Own_System       : Unsigned_8;
      Own_Component    : Unsigned_8;
      Target_System    : Unsigned_8;
      Target_Component : Unsigned_8;
      A                : out Action)
   with Pre  => Now >= Last_Time (D),
        Post => A.Kind = Send_Message
                and then State (D) = In_Progress
                and then not Item_Pending (D)
                and then Last_Time (D) = Now;

   --  Feed one received message (any ID) and check for a timeout.
   procedure Process
     (D : in out Download; Now : Milliseconds; M : Message; A : out Action)
   with Pre  => Now >= Last_Time (D),
        Post => Last_Time (D) = Now
                and then A.Kind in None_A | Send_Message | Got_Item
                and then (if A.Kind = Got_Item then Item_Pending (D));

   --  Check for a timeout without a new message.
   procedure Tick (D : in out Download; Now : Milliseconds; A : out Action)
   with Pre  => Now >= Last_Time (D),
        Post => Last_Time (D) = Now
                and then A.Kind in None_A | Send_Message;

   --  The item in the last Got_Item action has been stored. A requests
   --  the next item or, after the last one, is the final MISSION_ACK
   --  (State then reads Succeeded).
   procedure Item_Received
     (D : in out Download; Now : Milliseconds; A : out Action)
   with Pre  => Item_Pending (D) and then Now >= Last_Time (D),
        Post => A.Kind = Send_Message
                and then not Item_Pending (D)
                and then Last_Time (D) = Now;

private
   type Upload_Phase is
     (U_Idle, U_Waiting, U_Pending_Item, U_Done, U_Rejected, U_Timed_Out);

   type Upload is record
      Phase            : Upload_Phase := U_Idle;
      Count            : Unsigned_16 := 0;
      Own_System       : Unsigned_8 := 0;
      Own_Component    : Unsigned_8 := 0;
      Target_System    : Unsigned_8 := 0;
      Target_Component : Unsigned_8 := 0;
      --  The item the vehicle last requested, while an answer is pending.
      Pending_Seq      : Unsigned_16 := 0;
      --  Items 0 .. Next_Seq - 1 have been sent at least once.
      Next_Seq         : Unsigned_16 := 0;
      --  The last item sent, resent on a timeout once Next_Seq > 0.
      Last_Item        : Message;
      Result           : Unsigned_8 := 0;
      Attempts         : Natural range 0 .. Retry_Limit := 0;
      Sent_At          : Milliseconds := 0;
      Last_Now         : Milliseconds := 0;
      Sequence         : Unsigned_8 := 0;
   end record;

   type Download_Phase is
     (D_Idle, D_Waiting_Count, D_Waiting_Item, D_Pending_Item, D_Done,
      D_Rejected, D_Timed_Out);

   type Download is record
      Phase            : Download_Phase := D_Idle;
      Own_System       : Unsigned_8 := 0;
      Own_Component    : Unsigned_8 := 0;
      Target_System    : Unsigned_8 := 0;
      Target_Component : Unsigned_8 := 0;
      Count            : Unsigned_16 := 0;
      --  The item requested next; below Count while items are wanted.
      Expected_Seq     : Unsigned_16 := 0;
      Result           : Unsigned_8 := 0;
      Attempts         : Natural range 0 .. Retry_Limit := 0;
      Sent_At          : Milliseconds := 0;
      Last_Now         : Milliseconds := 0;
      Sequence         : Unsigned_8 := 0;
   end record;

   function Last_Time (U : Upload) return Milliseconds is (U.Last_Now);

   function State (U : Upload) return Status is
     (case U.Phase is
        when U_Idle => Idle,
        when U_Done => Succeeded,
        when U_Rejected => Rejected,
        when U_Timed_Out => Timed_Out,
        when U_Waiting | U_Pending_Item => In_Progress);

   function Item_Pending (U : Upload) return Boolean is
     (U.Phase = U_Pending_Item);

   function Result (U : Upload) return Unsigned_8 is (U.Result);

   function Last_Time (D : Download) return Milliseconds is (D.Last_Now);

   function State (D : Download) return Status is
     (case D.Phase is
        when D_Idle => Idle,
        when D_Done => Succeeded,
        when D_Rejected => Rejected,
        when D_Timed_Out => Timed_Out,
        when D_Waiting_Count | D_Waiting_Item | D_Pending_Item =>
          In_Progress);

   function Item_Pending (D : Download) return Boolean is
     (D.Phase = D_Pending_Item);

   function Count (D : Download) return Unsigned_16 is (D.Count);

   function Result (D : Download) return Unsigned_8 is (D.Result);

end MAVLink.Mission;
