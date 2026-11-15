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
kill $SIM 2>/dev/null || true
wait $SIM 2>/dev/null || true

run_tap() {
    "$BIN_DIR/hummingbird" --headless --duration 40 --imu icm42688p --gnss zed_f9p --gnss-rate 20 --mag rm3100 "$@" >/dev/null 2>&1 &
    SIM=$!
    attempt=0
    while [ $attempt -lt 60 ]; do
        PROBE=$("$BIN_DIR/sensor_tap" /drone 0.4 2>/dev/null || true)
        GNSS_SEQ=$(echo "$PROBE" | sed -n 's/.*gnss \([0-9][0-9]*\) baro.*/\1/p')
        if [ "${GNSS_SEQ:-0}" -ge 120 ]; then
            break
        fi
        attempt=$((attempt + 1))
    done
    "$BIN_DIR/sensor_tap" /drone 2
    kill $SIM 2>/dev/null || true
    wait $SIM 2>/dev/null || true
}
CALM=$(run_tap --weather clear_calm)
SHAKY=$(run_tap --weather hot_desert --vibration 3)
CALM_STD=$(echo "$CALM" | awk '/accel0 mean/ {print $5}')
SHAKY_STD=$(echo "$SHAKY" | awk '/accel0 mean/ {print $5}')
echo "accel0 std calm=$CALM_STD vibrating=$SHAKY_STD"
awk -v a="$CALM_STD" -v b="$SHAKY_STD" 'BEGIN { d = b * b - a * a; exit !(d > 0 && sqrt(d) > 0.05) }' || fail "vibration did not add accelerometer noise"
FX_BASE=$(run_tap --baro bmp388 | awk '/accel2 mean/ {print $3}')
FX_DEF=$(run_tap --baro bmp388 --fx-accel-deficit 1.0 --fx-baro-lag 0 --fx-cam-drop 0 --fx-cam-shift 0 | awk '/accel2 mean/ {print $3}')
echo "accel2 mean without=$FX_BASE with a 1.0 m/s^2 thrust deficit=$FX_DEF"
awk -v a="$FX_BASE" -v b="$FX_DEF" 'BEGIN { d = a - b; exit !(d > 0.15 && d < 0.5) }' || fail "the thrust-correlated accelerometer deficit is not applied at hover thrust"
if "$BIN_DIR/hummingbird" --headless --duration 1 --fx-accel-deficit 1.0 >/dev/null 2>&1; then
    fail "partial airframe effects were accepted"
fi
if "$BIN_DIR/hummingbird" --headless --duration 1 --fx-accel-deficit 1.0 --fx-baro-lag 0 --fx-cam-drop 0 --fx-cam-shift 0 >/dev/null 2>&1; then
    fail "an accelerometer deficit without an IMU profile was accepted"
fi
MAGZ=$(echo "$CALM" | awk '/mag2 mean/ {print $3}')
awk -v z="$MAGZ" 'BEGIN { exit !(z < -35 && z > -50) }' || fail "magnetometer vertical component $MAGZ outside the expected Earth field"
"$BIN_DIR/hummingbird" --headless --duration 15 --altitude 3 --rangefinder tf02_pro --rangefinder-rate 100 >/dev/null 2>&1 &
SIM=$!
sleep 4
IR=$("$BIN_DIR/sensor_tap" /drone 1.5)
kill $SIM 2>/dev/null || true
wait $SIM 2>/dev/null || true
echo "$IR" | grep "infrared:"
IR_MEAN=$(echo "$IR" | sed -n 's/.*infrared:.*valid, mean \([0-9.]*\) m.*/\1/p')
awk -v m="$IR_MEAN" 'BEGIN { exit !(m > 1.5 && m < 6.0) }' || fail "rangefinder reads $IR_MEAN m for a drone hovering near 3 m"
echo "$IR" | grep -q "limits 0.10..40.0 m" || fail "rangefinder limits do not match the tf02_pro profile"

CAM_LOG="$SHM/camera_sim.log"
"$BIN_DIR/hummingbird" --headless --duration 60 --cam-width 1280 --cam-height 720 --cam-fps 120 \
    --cam-profile ar0234 >"$CAM_LOG" 2>&1 &
SIM=$!
sleep 1.5
CAM=""
for attempt in 1 2 3 4 5 6 7 8; do
    CAM=$("$BIN_DIR/sensor_tap" /drone 3 "$BIN_DIR/test_frame.pgm" || true)
    if echo "$CAM" | grep -q "^camera: "; then
        break
    fi
    if grep -q "headless camera unavailable\|no usable EGL device\|built without EGL" "$CAM_LOG"; then
        break
    fi
    sleep 1
done
kill $SIM 2>/dev/null || true
wait $SIM 2>/dev/null || true
if grep -q "headless camera unavailable\|no usable EGL device\|built without EGL" "$CAM_LOG"; then
    echo "SKIP camera checks: no EGL device on this machine"
else
    echo "$CAM" | grep "camera:" || fail "the camera ring never came up"
    echo "$CAM" | grep -q "1280x720 x1" || fail "camera frames are not 1280x720 mono8"
    CAM_FRAMES=$(echo "$CAM" | sed -n 's/^camera: \([0-9]*\) frames.*/\1/p')
    CAM_RATE=$(echo "$CAM" | sed -n 's/.*simulated-time rate \([0-9.]*\) fps.*/\1/p')
    [ "${CAM_FRAMES:-0}" -ge 2 ] || fail "camera delivered fewer than two frames"
    if [ "$CAM_FRAMES" -ge 50 ]; then
        CAM_TOL=1.2
    else
        CAM_TOL=6.0
    fi
    awk -v r="$CAM_RATE" -v t="$CAM_TOL" 'BEGIN { exit !(r >= 120 - t && r <= 120 + t) }' ||
        fail "camera rate $CAM_RATE fps is outside $CAM_TOL fps of 120 over $CAM_FRAMES frames"
    CAM_MEAN=$(echo "$CAM" | sed -n 's/.*mean level \([0-9.]*\).*/\1/p')
    awk -v m="$CAM_MEAN" 'BEGIN { exit !(m > 5 && m < 250) }' || fail "camera image is blank or saturated (mean $CAM_MEAN)"
    [ "$(wc -c < "$BIN_DIR/test_frame.pgm")" -gt 921600 ] || fail "frame dump has the wrong size"
fi
rm -f "$BIN_DIR/test_frame.pgm"
echo "PASS"
