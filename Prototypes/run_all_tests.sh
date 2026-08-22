#!/usr/bin/env bash
#
# Build and run every assert suite under Prototypes/.
#
# There are thirty-six of them now, in seven directories, and the per-suite
# command in PROTOTYPES.md is still the right way to iterate on ONE of them.
# This is for the other question — "is all of it still green" — which is the one
# nobody asks often enough by hand, and the one CI asks every time.
#
#   ./run_all_tests.sh              # clang++ if it is there, else g++
#   CXX=g++ ./run_all_tests.sh      # or say which
#
# Exits non-zero if anything fails to build or fails an assertion, so it can be
# a CI step as it stands.
#
# RUN FROM EACH SUITE'S OWN DIRECTORY, because they include their headers by
# relative path and a couple read fixture files beside them. That is why this
# cds rather than passing paths.

set -u

CXX="${CXX:-}"
if [ -z "$CXX" ]; then
    if command -v clang++ >/dev/null 2>&1; then CXX=clang++
    elif command -v g++ >/dev/null 2>&1; then CXX=g++
    else echo "no clang++ or g++ on PATH"; exit 127
    fi
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="${TMPDIR:-/tmp}/tu-prototype-tests"
mkdir -p "$BUILD"

PASS=0
FAIL=0
FAILED=""

# Sorted, so the output is the same run to run and a diff of two runs is
# readable. Deterministic order matters more than speed here.
for SRC in $(cd "$HERE" && find . -name 'test_*.cpp' | sort); do
    DIR="$(dirname "$SRC")"
    NAME="$(basename "$SRC" .cpp)"
    printf '%-28s ' "$NAME"

    if ! (cd "$HERE/$DIR" && "$CXX" -std=c++17 -Wall -Wextra -O2 \
            -o "$BUILD/$NAME" "$NAME.cpp") > "$BUILD/$NAME.build" 2>&1; then
        echo "BUILD FAILED"
        sed 's/^/    /' "$BUILD/$NAME.build"
        FAIL=$((FAIL + 1)); FAILED="$FAILED $NAME"
        continue
    fi

    if (cd "$HERE/$DIR" && "$BUILD/$NAME") > "$BUILD/$NAME.log" 2>&1; then
        echo "ok"
        PASS=$((PASS + 1))
    else
        echo "FAILED"
        # The last lines are the ones with the assertion in them. Printed here
        # rather than left in a file, because a runner that only says how many
        # failed sends you looking for the log by hand every time.
        tail -12 "$BUILD/$NAME.log" | sed 's/^/    /'
        FAIL=$((FAIL + 1)); FAILED="$FAILED $NAME"
    fi
done

echo
echo "$PASS passed, $FAIL failed"
if [ "$FAIL" -ne 0 ]; then
    echo "failed:$FAILED"
    exit 1
fi
