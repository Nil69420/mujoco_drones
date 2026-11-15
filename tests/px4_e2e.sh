#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
if [ -z "${PX4_BIN:-}" ] || [ ! -x "${PX4_BIN}" ]; then
    echo "SKIP: set PX4_BIN to a px4 SITL binary built without Gazebo (px4_sitl_nogz or default)"
    exit 0
fi
PX4_ETC=${PX4_ETC:-$(dirname "$PX4_BIN")/../etc}
if [ ! -d "$PX4_ETC/init.d-posix" ]; then
    echo "SKIP: set PX4_ETC to the etc directory of the PX4 build"
    exit 0
fi
PY=${PX4_PYTHON:-python3}
if ! "$PY" -c "import pymavlink" 2>/dev/null; then
    echo "SKIP: pymavlink not importable by $PY"
    exit 0
fi
SHM=$(mktemp -d)
WORK=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
SIM_PID=""
PX4_PID=""
cleanup() {
    [ -n "$PX4_PID" ] && kill "$PX4_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [ -n "$PX4_PID" ] && kill -9 "$PX4_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    rm -rf "$SHM"
    if [ -n "${PX4_E2E_KEEP:-}" ]; then echo "logs kept in $WORK"; else rm -rf "$WORK"; fi
}
trap cleanup EXIT
mkdir -p "$WORK/rootfs"
cp -r "$PX4_ETC" "$WORK/etc"
cp "$SRC_DIR/tests/px4/10099_mujoco_plus_quad" "$WORK/etc/init.d-posix/airframes/"
( cd "$SRC_DIR" && exec env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --scenario scenarios/px4_quad.scn ) > "$WORK/sim.log" 2>&1 &
SIM_PID=$!
sleep 2
( cd "$WORK/rootfs" && exec env PX4_SIM_MODEL=mujoco_plus_quad "$PX4_BIN" -d "$WORK/etc" ) > "$WORK/px4.log" 2>&1 &
PX4_PID=$!
sleep 5
"$PY" -u -I "$SRC_DIR/tests/px4_e2e.py" udpin:0.0.0.0:14550
