#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
SCENARIO=$3
SECONDS_SIM=${4:-5}
SHM=$(mktemp -d)
trap 'rm -rf "$SHM"' EXIT
export RENOIR_SHM_DIR=$SHM
OUT=$(cd "$SRC_DIR" && env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --scenario "$SCENARIO" --duration "$SECONDS_SIM" 2>&1) || {
    echo "$OUT" | tail -20
    echo "FAIL: $SCENARIO exited with an error"
    exit 1
}
echo "$OUT" | grep -q "final states" || { echo "FAIL: $SCENARIO printed no final states"; exit 1; }
if echo "$OUT" | grep -qiE "\bnan\b|\binf\b"; then
    echo "$OUT" | grep -iE "\bnan\b|\binf\b" | head -3
    echo "FAIL: $SCENARIO produced a non-finite value"
    exit 1
fi
echo "PASS $SCENARIO"
