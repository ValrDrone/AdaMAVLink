#!/bin/sh
# Builds the library (both modes) and runs the tests with GNAT 14.2, the
# oldest supported compiler, in a throwaway Alire settings folder so the
# pinned toolchain in tests/alire.toml stays untouched. Run in the
# container (make test-gnat14).
set -eu
settings=$(mktemp -d)
build=$(mktemp -d)
alr="alr -n -s $settings"
$alr toolchain --select gnat_native=14.2.1 gprbuild=26.0.1 >/dev/null
for mode in dev release; do
    $alr exec -- gprbuild -q -p -P adamavlink.gpr \
        -XADAMAVLINK_BUILD_MODE=$mode --relocate-build-tree="$build"
done
$alr exec -- gprbuild -q -p -aP . -P tests/adamavlink_tests.gpr \
    -XADAMAVLINK_BUILD_MODE=dev --relocate-build-tree="$build" --root-dir=.
for test in test_mavlink test_messages test_envelope test_mission; do
    "$(find "$build" -name $test -type f)"
done
rm -rf "$settings" "$build"
