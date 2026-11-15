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
SHM=$(mktemp -d)
WORK=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
SIM_PID=""
AP_PID=""
cleanup() {
    [ -n "$AP_PID" ] && kill "$AP_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [ -n "$AP_PID" ] && kill -9 "$AP_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    rm -rf "$SHM"
    if [ -n "${ARDUPILOT_E2E_KEEP:-}" ]; then echo "logs kept in $WORK"; else rm -rf "$WORK"; fi
}
trap cleanup EXIT
cp "$DEFAULTS" "$WORK/defaults.parm"
printf "\nFRAME_CLASS 1\nFRAME_TYPE 6\nARMING_CHECK 0\n" >> "$WORK/defaults.parm"
( cd "$SRC_DIR" && exec env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --scenario scenarios/ardupilot_quad.scn ) > "$WORK/sim.log" 2>&1 &
SIM_PID=$!
cd "$WORK"
sleep 2
"$ARDUPILOT_COPTER" -S --model JSON:127.0.0.1 --speedup 1 --defaults defaults.parm \
    --home -35.363261,149.165230,584,353 > ap.log 2>&1 &
AP_PID=$!
sleep 3
"$PY" -I "$SRC_DIR/tests/ardupilot_e2e.py" tcp:127.0.0.1:5760
