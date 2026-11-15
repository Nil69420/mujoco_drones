#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
if [ -z "${ARDUPILOT_COPTER:-}" ] || [ ! -x "${ARDUPILOT_COPTER}" ]; then
    echo "SKIP: set ARDUPILOT_COPTER to a SITL arducopter binary"
    exit 0
fi
PY=${ARDUPILOT_PYTHON:-python3}
if ! "$PY" -c "import pymavlink" 2>/dev/null; then
    echo "SKIP: pymavlink not importable by $PY"
    exit 0
fi
DEFAULTS=${ARDUPILOT_COPTER_DEFAULTS:-}
if [ -z "$DEFAULTS" ] || [ ! -f "$DEFAULTS" ]; then
    echo "SKIP: set ARDUPILOT_COPTER_DEFAULTS to ArduPilot's copter.parm"
    exit 0
fi
SIM_PID=""
AP_PID=""
SHM=""
WORK=""
stop_all() {
    [ -n "$AP_PID" ] && kill "$AP_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [ -n "$AP_PID" ] && kill -9 "$AP_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    AP_PID=""
    SIM_PID=""
    [ -n "$SHM" ] && rm -rf "$SHM"
    [ -n "$WORK" ] && rm -rf "$WORK"
    SHM=""
    WORK=""
}
trap stop_all EXIT
fly() {
    SHM=$(mktemp -d)
    WORK=$(mktemp -d)
    export RENOIR_SHM_DIR=$SHM
    cp "$DEFAULTS" "$WORK/defaults.parm"
    printf "\nFRAME_CLASS 1\nFRAME_TYPE 6\nARMING_CHECK 0\n" >> "$WORK/defaults.parm"
    ( cd "$SRC_DIR" && exec env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --scenario "$1" ) > "$WORK/sim.log" 2>&1 &
    SIM_PID=$!
    ( cd "$WORK" && sleep 2 && exec "$ARDUPILOT_COPTER" -S --model JSON:127.0.0.1 --speedup 1 \
        --defaults defaults.parm --home -35.363261,149.165230,584,353 ) > "$WORK/ap.log" 2>&1 &
    AP_PID=$!
    sleep 5
    "$PY" -I "$SRC_DIR/tests/ardupilot_effects_e2e.py" tcp:127.0.0.1:5760 | grep "^HOVER"
    stop_all
}
BASE=$(fly scenarios/ardupilot_quad_model.scn) || { echo "FAIL: baseline flight"; exit 1; }
FX=$(fly scenarios/ardupilot_quad_effects.scn) || { echo "FAIL: effects flight"; exit 1; }
echo "baseline: $BASE"
echo "deficit:  $FX"
echo "$BASE $FX" | awk '{
    dz = $10 - $3; da = $12 - $5;
    printf "zacc shift %.1f mG, hold altitude shift %.3f m\n", dz, da;
    exit !(dz > 15 && dz < 45 && da > 0.1 && da < 1.0)
}' || { echo "FAIL: the deficit did not reach ArduPilot or did not disturb the flight"; exit 1; }
echo "PASS"
