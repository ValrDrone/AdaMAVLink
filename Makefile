.DEFAULT_GOAL := help
.PHONY: setup generate check-generated build test test-gnat14 prove vectors vectors-messages vectors-enums ci clean help

COMPOSE := docker compose
ADA := $(COMPOSE) run --rm ada-dev
# Tests and proof run in the tests crate, which pins the toolchain and
# depends on the library by path.
IN_TESTS := sh -c 'cd tests && alr -n exec -- $$0 "$$@"'

setup:
	$(COMPOSE) build ada-dev
	$(ADA) sh -c 'cd tests && alr -n update'

# The message codecs and the dialect table from definitions/ and
# generator/messages.txt.
GENERATE := python3 generator/adamavlink_gen.py definitions/common.xml generator/messages.txt src docs/messages.md

generate:
	$(ADA) $(GENERATE)

# The committed code is what the generator writes (CI).
check-generated:
	$(ADA) sh -c '$(GENERATE) && git diff --exit-code -- src docs/messages.md'

build:
	$(ADA) $(IN_TESTS) gprbuild -p -P ../adamavlink.gpr -XADAMAVLINK_BUILD_MODE=dev

test:
	$(ADA) $(IN_TESTS) gprbuild -p -P adamavlink_tests.gpr -XADAMAVLINK_BUILD_MODE=dev
	$(ADA) sh -c 'tests/bin/test_mavlink && tests/bin/test_messages && tests/bin/test_enums && tests/bin/test_statustext && tests/bin/test_envelope'

# The oldest supported compiler (scripts/test-gnat14.sh).
test-gnat14:
	$(ADA) sh scripts/test-gnat14.sh

prove:
	$(ADA) $(IN_TESTS) gnatprove -P ../adamavlink_proof.gpr --mode=prove --level=2 --checks-as-errors=on --warnings=error

# Regenerates the expected frames in the tests from pymavlink (needs uv on
# the host); compare its output with tests/src/test_mavlink.adb.
vectors:
	uv run --no-project --with pymavlink==2.4.50 python -I tests/vectors/generate.py

# Rewrites tests/src/test_messages.adb from pymavlink (needs uv on the host).
vectors-messages:
	uv run --no-project --with pymavlink==2.4.50 python -I tests/vectors/generate_messages.py

# Rewrites the enum test from the definitions, cross-checked against
# pymavlink's published constants.
vectors-enums:
	uv run --no-project --with pymavlink==2.4.50 python -I tests/vectors/generate_enums.py

ci: check-generated build test test-gnat14 prove

clean:
	rm -rf obj lib tests/obj tests/bin

help:
	@echo "make setup    build the development image and fetch the toolchain"
	@echo "make generate write the message codecs from definitions/"
	@echo "make build    build the library (dev mode: warnings are errors)"
	@echo "make test     build and run the tests"
	@echo "make test-gnat14  build and test with GNAT 14.2"
	@echo "make prove    prove the library with GNATprove"
	@echo "make vectors  print the frame test's pymavlink vectors"
	@echo "make vectors-messages  rewrite the message test from pymavlink"
	@echo "make vectors-enums  rewrite the enum test from pymavlink"
	@echo "make ci       check generated code, build, test and prove"
