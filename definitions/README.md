# MAVLink message definitions

`minimal.xml`, `standard.xml` and `common.xml` are the MAVLink message
definitions from the MAVLink project (github.com/mavlink/mavlink) as shipped
in pymavlink 2.4.50 (`pymavlink/message_definitions/v1.0`), unchanged apart
from line endings (CRLF there, LF here). The
MAVLink project releases its XML message definitions under the MIT licence.

`generator/adamavlink_gen.py` reads them; nothing else in the library
depends on them at build time.

SHA-256 (with LF line endings):

```
b3627df397e3354f26efe63559e1bcec4e72a4e4562ab52cbee1bab8f9e9999e  minimal.xml
1c92c50e0a4d4cc29e4125dff33b16062e1e285a6271292fb3a4266576487e65  standard.xml
fbac92d0689c9b1bc03b49ebc5cdf989f3fc23026c3c856f7224e4b9af1b5f53  common.xml
```

To update: copy the files from a newer pymavlink (or the MAVLink repository
at a tagged commit), update the hashes and the version above, run
`make generate vectors-messages`, and check the test and proof still pass.
