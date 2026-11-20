# MAVLink message definitions

`minimal.xml`, `standard.xml` and `common.xml` are the MAVLink message
definitions from the MAVLink project (github.com/mavlink/mavlink), unchanged,
as shipped in pymavlink 2.4.50 (`pymavlink/message_definitions/v1.0`). The
MAVLink project releases its XML message definitions under the MIT licence.

`generator/adamavlink_gen.py` reads them; nothing else in the library
depends on them at build time.

SHA-256:

```
5b450b2eef9e93db9c0a35ea56141bd05c197f41a9f4a7990a99b91c16814fa9  minimal.xml
d41d5a29619345fcaa110891682e6df83eca081168f76ecfd17d59d0ea721c47  standard.xml
ec9a32b00ed801ed6231edd0da943b782b7e5070b1134e8738a8ada0949ee2cb  common.xml
```

To update: copy the files from a newer pymavlink (or the MAVLink repository
at a tagged commit), update the hashes and the version above, run
`make generate vectors-messages`, and check the test and proof still pass.
