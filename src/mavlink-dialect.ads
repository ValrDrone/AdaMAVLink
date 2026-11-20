pragma SPARK_Mode (On);

--  The messages this build knows, from the MAVLink message definitions
--  (common.xml and minimal.xml). A frame for a message not listed here
--  cannot have its checksum checked and is reported, not delivered.
--
--  Kept by hand until the generator (docs/design.md, Roadmap) writes this
--  table and the message codecs from the definitions.

package MAVLink.Dialect is
   Heartbeat_Id : constant Message_Id := 0;

   function Info (Id : Message_Id) return Message_Info
   is (case Id is
         when Heartbeat_Id =>
           (Known => True, CRC_Extra => 50, Max_Length => 9),
         when others       => (others => <>));

   function Is_Known (Id : Message_Id) return Boolean is (Info (Id).Known);
end MAVLink.Dialect;
