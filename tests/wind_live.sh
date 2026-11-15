#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
SHM=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
WORK=$(mktemp -d)
trap 'rm -rf "$SHM" "$WORK"' EXIT
fail() { echo "FAIL: $1"; exit 1; }

hover_displacement() {
    cat > "$WORK/$1.scn" <<SCN
[scenario]
environment = desert
environment_seed = 5
weather = $1
duration = 20.0

[vehicle w0]
airframe = quad_x
controller = pd_multicopter
estimator = kf_ins
localization = gnss
spawn = 0.0 0.0 0.3
target = 0.0 0.0 10.0
yaw = 0.0
imu = icm42688p
gnss = ideal
baro = ideal
mag = ideal
SCN
    (cd "$SRC_DIR" && "$BIN_DIR/hummingbird" --headless --scenario "$WORK/$1.scn" 2>&1) |
        awk '/t= *19\.[0-9]0 / { sub(/.*pos=\(/, ""); sub(/\).*/, ""); print sqrt($1*$1 + $2*$2); exit }'
}

CALM=$(hover_displacement clear_calm)
STORM=$(hover_displacement storm)
echo "horizontal displacement after 19 s: clear_calm=$CALM m storm=$STORM m"
[ -n "$CALM" ] && [ -n "$STORM" ] || fail "no telemetry"
awk -v c="$CALM" 'BEGIN { exit !(c < 2.0) }' || fail "the drone does not hold position in calm air"
awk -v c="$CALM" -v s="$STORM" 'BEGIN { exit !(s > 20 * c && s > 30) }' || fail "wind does not push the drone"
echo "PASS"
