#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
SHM=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
WORK=$(mktemp -d)
trap 'rm -rf "$SHM" "$WORK"' EXIT
fail() { echo "FAIL: $1"; exit 1; }

python3 -I "$SRC_DIR/tools/gen_multirotor.py" "$WORK"
for name in hexa octo; do
    cmp -s "$WORK/$name.xml" "$SRC_DIR/model/airframes/$name.xml" || fail "$name.xml differs from tools/gen_multirotor.py output"
done

fly() {
    sed "s/airframe = .*/airframe = $1/" "$SRC_DIR/scenarios/hexa_hover.scn" > "$WORK/$1.scn"
    (cd "$SRC_DIR" && "$BIN_DIR/hummingbird" --headless --scenario "$WORK/$1.scn" 2>&1) |
        awk '/final states/ { found = 1; next } found && /pos=/ { sub(/.*pos=\(/, ""); sub(/\).*/, ""); print; exit }'
}

for name in quad_x hexa octo; do
    pos=$(fly "$name")
    [ -n "$pos" ] || fail "$name produced no final state"
    echo "$name final position: $pos"
    echo "$pos" | awk '{ horizontal = sqrt($1*$1 + $2*$2); dz = $3 - 3.0; if (dz < 0) dz = -dz; exit !(horizontal < 3.0 && dz < 1.0 && $3 == $3) }' || fail "$name does not hold the target"
done
echo "PASS"
