pragma SPARK_Mode (On);

package body MAVLink.Heartbeat is
   function Pack (C : Content) return Payload is
      Mode : constant Unsigned_32 := C.Custom_Mode;
   begin
      return
        [1 => Unsigned_8 (Mode and 16#FF#),
         2 => Unsigned_8 (Shift_Right (Mode, 8) and 16#FF#),
         3 => Unsigned_8 (Shift_Right (Mode, 16) and 16#FF#),
         4 => Unsigned_8 (Shift_Right (Mode, 24)),
         5 => C.Vehicle_Type,
         6 => C.Autopilot,
         7 => C.Base_Mode,
         8 =>
           (if C.State_Known then System_State'Pos (C.State) else 255),
         9 => C.Version];
   end Pack;

   function Unpack (M : Message) return Content is
      P     : Payload_Bytes renames M.Payload;
      State : constant Unsigned_8 := P (8);
   begin
      return
        (Custom_Mode  =>
           Unsigned_32 (P (1))
           or Shift_Left (Unsigned_32 (P (2)), 8)
           or Shift_Left (Unsigned_32 (P (3)), 16)
           or Shift_Left (Unsigned_32 (P (4)), 24),
         Vehicle_Type => P (5),
         Autopilot    => P (6),
         Base_Mode    => P (7),
         State_Known  => State <= System_State'Pos (System_State'Last),
         State        =>
           (if State <= System_State'Pos (System_State'Last)
            then System_State'Val (State)
            else Uninitialised),
         Version      => P (9));
   end Unpack;
end MAVLink.Heartbeat;
