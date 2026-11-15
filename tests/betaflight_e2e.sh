#!/bin/sh
set -eu
BIN_DIR=$1
SRC_DIR=$2
if [ -z "${BETAFLIGHT_SITL:-}" ] || [ ! -x "${BETAFLIGHT_SITL}" ]; then
    echo "SKIP: set BETAFLIGHT_SITL to a Betaflight SITL binary (make TARGET=SITL)"
    exit 0
fi
PY=${BETAFLIGHT_PYTHON:-python3}
SHM=$(mktemp -d)
WORK=$(mktemp -d)
export RENOIR_SHM_DIR=$SHM
SIM_PID=""
BF_PID=""
cleanup() {
    [ -n "$BF_PID" ] && kill "$BF_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [ -n "$BF_PID" ] && kill -9 "$BF_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    rm -rf "$SHM"
    if [ -n "${BETAFLIGHT_E2E_KEEP:-}" ]; then echo "logs kept in $WORK"; else rm -rf "$WORK"; fi
}
trap cleanup EXIT
start_bf() {
    ( cd "$WORK" && exec stdbuf -oL "$BETAFLIGHT_SITL" ) > "$WORK/bf.log" 2>&1 &
    BF_PID=$!
}
( cd "$SRC_DIR" && exec env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --scenario scenarios/betaflight_quad.scn ) > "$WORK/sim.log" 2>&1 &
SIM_PID=$!
sleep 2
port_open() {
    "$PY" -I -c "import socket,sys
try:
    socket.create_connection(('127.0.0.1', 5761), timeout=1).close()
except OSError:
    sys.exit(1)"
}
wait_for_bf() {
    for _ in $(seq 1 40); do
        if [ -z "$BF_PID" ] || ! kill -0 "$BF_PID" 2>/dev/null; then
            start_bf
        fi
        if port_open; then
            return 0
        fi
        sleep 1
    done
    echo "FAIL: Betaflight never opened its MSP port"
    exit 1
}
wait_for_bf
sleep 3
"$PY" -I "$SRC_DIR/tests/betaflight_e2e.py" configure "$SRC_DIR/tests/betaflight/config.cli"
sleep 3
kill "$BF_PID" 2>/dev/null || true
BF_PID=""
for _ in $(seq 1 20); do
    port_open || break
    sleep 0.5
done
sleep 1
wait_for_bf
if ! "$PY" -u -I "$SRC_DIR/tests/betaflight_e2e.py" fly "$WORK/sim.log" "${BETAFLIGHT_E2E_MODE:-flight}"; then
    echo "--- simulator log (tail) ---"
    tail -n 30 "$WORK/sim.log" || true
    echo "--- betaflight log (last arming state) ---"
    grep "Arming" "$WORK/bf.log" | tail -n 3 || true
    tail -n 5 "$WORK/bf.log" || true
    exit 1
fi
