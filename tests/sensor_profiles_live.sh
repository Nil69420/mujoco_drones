#!/bin/sh
set -eu
BIN_DIR=$1
SHM=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
SIM=""
trap 'rm -rf "$SHM"; [ -n "$SIM" ] && kill $SIM 2>/dev/null || true' EXIT
fail() { echo "FAIL: $1"; exit 1; }

if "$BIN_DIR/hummingbird" --headless --duration 1 --imu does_not_exist >/dev/null 2>&1; then
    fail "unknown IMU profile was accepted"
fi
if "$BIN_DIR/hummingbird" --headless --duration 1 --imu icm42688p --imu-rate 2000 >/dev/null 2>&1; then
    fail "an IMU rate above the simulation step rate was accepted"
fi

"$BIN_DIR/hummingbird" --headless --duration 40 --imu mpu6050 --gnss zed_f9p --gnss-rate 20 \
    --baro bmp388 --baro-rate 100 >/dev/null 2>&1 &
SIM=$!
sleep 0.5
OUT=$("$BIN_DIR/sensor_tap" /drone 1.5)
echo "$OUT"
echo "$OUT" | grep -q "simulated-time rates: imu 1000.0 Hz, gnss 20.0 Hz, baro 100.0 Hz" ||
    fail "published rates differ from the configured rates"
echo "$OUT" | grep -q "accel2 mean" || fail "no IMU statistics"
echo "PASS"
