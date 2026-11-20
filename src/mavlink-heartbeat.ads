pragma SPARK_Mode (On);

with MAVLink.Dialect;

--  HEARTBEAT (message 0, minimal.xml): every MAVLink system sends one at
--  least once a second; a ground station lists a vehicle by its heartbeats
--  and reads its type, autopilot, mode and state from them.
--
--  Payload, 9 bytes: custom_mode (uint32), type, autopilot, base_mode,
--  system_status, mavlink_version (uint8 each).

package MAVLink.Heartbeat is
   Id : constant Message_Id := Dialect.Heartbeat_Id;

   subtype Payload is Byte_Array (1 .. 9);

   --  MAV_TYPE values used here.
   Type_Generic      : constant Unsigned_8 := 0;
   Type_Quadrotor    : constant Unsigned_8 := 2;
   Type_GCS          : constant Unsigned_8 := 6;
   Type_Ground_Rover : constant Unsigned_8 := 10;

   --  MAV_AUTOPILOT values used here.
   Autopilot_Generic : constant Unsigned_8 := 0;
   Autopilot_Invalid : constant Unsigned_8 := 8;

   --  MAV_MODE_FLAG bits of base_mode.
   Mode_Custom_Enabled : constant Unsigned_8 := 16#01#;
   Mode_Test           : constant Unsigned_8 := 16#02#;
   Mode_Auto           : constant Unsigned_8 := 16#04#;
   Mode_Guided         : constant Unsigned_8 := 16#08#;
   Mode_Stabilize      : constant Unsigned_8 := 16#10#;
   Mode_HIL            : constant Unsigned_8 := 16#20#;
   Mode_Manual_Input   : constant Unsigned_8 := 16#40#;
   Mode_Safety_Armed   : constant Unsigned_8 := 16#80#;

   --  MAV_STATE.
   type System_State is
     (Uninitialised,
      Boot,
      Calibrating,
      Standby,
      Active,
      Critical,
      Emergency,
      Power_Off,
      Flight_Termination);

   --  The protocol version field: 3 for MAVLink 2.
   Protocol_Version : constant Unsigned_8 := 3;

   type Content is record
      Custom_Mode     : Unsigned_32 := 0;
      Vehicle_Type    : Unsigned_8 := Type_Generic;
      Autopilot       : Unsigned_8 := Autopilot_Generic;
      Base_Mode       : Unsigned_8 := 0;
      --  Unknown when the sender's value is outside MAV_STATE.
      State_Known     : Boolean := True;
      State           : System_State := Uninitialised;
      Version         : Unsigned_8 := Protocol_Version;
   end record;

   function Pack (C : Content) return Payload;

   --  The heartbeat in a received message M for this ID.
   function Unpack (M : Message) return Content
   with Pre => M.Head.Id = Id;
end MAVLink.Heartbeat;
