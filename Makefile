.DEFAULT_GOAL := help
.PHONY: setup build test test-gnat14 prove vectors ci clean help

COMPOSE := docker compose
ADA := $(COMPOSE) run --rm ada-dev
# Tests and proof run in the tests crate, which pins the toolchain and
# depends on the library by path.
IN_TESTS := sh -c 'cd tests && alr -n exec -- $$0 "$$@"'

setup:
	$(COMPOSE) build ada-dev
	$(ADA) sh -c 'cd tests && alr -n update'

build:
	$(ADA) $(IN_TESTS) gprbuild -p -P ../adamavlink.gpr -XADAMAVLINK_BUILD_MODE=dev

test:
	$(ADA) $(IN_TESTS) gprbuild -p -P adamavlink_tests.gpr -XADAMAVLINK_BUILD_MODE=dev
	$(ADA) tests/bin/test_mavlink

# The oldest supported compiler (scripts/test-gnat14.sh).
test-gnat14:
	$(ADA) sh scripts/test-gnat14.sh

prove:
	$(ADA) $(IN_TESTS) gnatprove -P ../adamavlink_proof.gpr --mode=prove --level=2 --checks-as-errors=on --warnings=error

# Regenerates the expected frames in the tests from pymavlink (needs uv on
# the host); compare its output with tests/src/test_mavlink.adb.
vectors:
	uv run --no-project --with pymavlink==2.4.50 python -I tests/vectors/generate.py

ci: build test test-gnat14 prove

clean:
	rm -rf obj lib tests/obj tests/bin

help:
	@echo "make setup    build the development image and fetch the toolchain"
	@echo "make build    build the library (dev mode: warnings are errors)"
	@echo "make test     build and run the tests"
	@echo "make test-gnat14  build and test with GNAT 14.2"
	@echo "make prove    prove the library with GNATprove"
	@echo "make vectors  print the pymavlink test vectors"
	@echo "make ci       build, test and prove"
