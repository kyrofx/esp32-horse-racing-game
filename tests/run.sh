#!/bin/sh
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
scratch=$(mktemp -d "${TMPDIR:-/tmp}/race-diagnostics.XXXXXX")
trap 'rm -rf "$scratch"' EXIT
trap 'exit 1' HUP INT TERM

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -g \
  -I "$root/shared/src" "$root/tests/diagnostics_test.cpp" \
  "$root/shared/src/race/Link.cpp" "$root/shared/src/race/MotorTrial.cpp" "$root/shared/src/race/ButtonPairing.cpp" \
  -o "$scratch/diagnostics_test"
"$scratch/diagnostics_test"
