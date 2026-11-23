with Ada.Assertions;
with Ada.Text_IO;
with Interfaces;
with MAVLink.Messages.Statustext;

--  The STATUSTEXT string accessors (MAVLink.Messages.Statustext): NUL-
--  trimmed reads, NUL-padded writes, the length precondition on writes and
--  slices whose first position is not one.
procedure Test_Statustext is
   use type Interfaces.Unsigned_8;
   package Statustext renames MAVLink.Messages.Statustext;

   procedure Check (Condition : Boolean; Message : String) is
   begin
      if not Condition then
         raise Program_Error with Message;
      end if;
   end Check;

   C        : Statustext.Content;
   Full_50  : constant String :=
     "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
     & "abcdefghijklmnopqrstuvwx"; -- 26 + 24 = 50
   pragma Assert (Full_50'Length = 50);
   World    : constant String := "HELLO WORLD";

begin
   --  A default content is empty.
   Check (Statustext.Text_Length (C) = 0, "default length");
   Check (Statustext.Text (C) = "", "default text");

   --  A short string: trimmed read back, the rest NUL.
   Statustext.Put_Text (C, "hello");
   Check (Statustext.Text_Length (C) = 5, "short length");
   Check (Statustext.Text (C) = "hello", "short text");
   Check (C.Text (6) = 16#00#, "short padding");

   --  The full field.
   Statustext.Put_Text (C, Full_50);
   Check (Statustext.Text_Length (C) = 50, "full length");
   Check (Statustext.Text (C) = Full_50, "full text");

   --  The empty string.
   Statustext.Put_Text (C, "");
   Check (Statustext.Text_Length (C) = 0, "empty length");
   Check (C.Text (1) = 16#00#, "empty padding");

   --  A slice whose first position is not one.
   Statustext.Put_Text (C, World (3 .. 7));
   Check (Statustext.Text_Length (C) = 5, "slice length");
   Check (Statustext.Text (C) = "LLO W", "slice text");

   --  The length precondition rejects a string that does not fit.
   declare
      Raised : Boolean := False;
   begin
      begin
         Statustext.Put_Text (C, Full_50 & "!");
         Raised := True; -- reached only if no exception was raised
      exception
         when Ada.Assertions.Assertion_Error =>
            null; -- expected: the precondition is enforced at run time
      end;
      Check (not Raised, "Put_Text accepted 51 characters");
   end;

   Ada.Text_IO.Put_Line ("STATUSTEXT string accessor tests passed");
end Test_Statustext;
