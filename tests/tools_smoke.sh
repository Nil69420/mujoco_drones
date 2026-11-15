#!/bin/sh
set -eu
BIN_DIR=$1
fail() { echo "FAIL: $1"; exit 1; }
status_of() { "$@" >/dev/null 2>&1 && echo 0 || echo $?; }

OUT=$("$BIN_DIR/pilot_tool" 2>&1 || true)
echo "$OUT" | grep -q "usage: pilot_tool" || fail "pilot_tool printed no usage without arguments"
[ "$(status_of "$BIN_DIR/pilot_tool")" = 2 ] || fail "pilot_tool without arguments did not exit with 2"
"$BIN_DIR/pilot_tool" profiles | grep -q "DualSense" || fail "pilot_tool profiles lists no DualSense profile"
[ "$(status_of "$BIN_DIR/pilot_tool" list)" = 0 ] || fail "pilot_tool list failed on a machine with no controllers"
for sub in "monitor /nonexistent" "calibrate /nonexistent out.profile" "betaflight /nonexistent"; do
    # shellcheck disable=SC2086
    OUT=$("$BIN_DIR/pilot_tool" $sub 2>&1 || true)
    echo "$OUT" | grep -q "cannot open /nonexistent" || fail "pilot_tool $sub did not report the missing device"
    # shellcheck disable=SC2086
    [ "$(status_of "$BIN_DIR/pilot_tool" $sub)" = 1 ] || fail "pilot_tool $sub did not exit with 1"
done

"$BIN_DIR/send_command" --help | grep -q "Publish a single command setpoint" || fail "send_command --help is wrong"
[ "$(status_of "$BIN_DIR/send_command" --mode 999)" = 1 ] || fail "an out-of-range --mode was accepted"
[ "$(status_of "$BIN_DIR/send_command" --mode x)" = 1 ] || fail "a non-numeric --mode was accepted"
[ "$(status_of "$BIN_DIR/send_command" --bogus)" = 1 ] || fail "an unknown option was accepted"

SHM=$(mktemp -d)
SIM=""
trap '[ -n "$SIM" ] && kill "$SIM" 2>/dev/null || true; rm -rf "$SHM"' EXIT
export RENOIR_SHM_DIR=$SHM
env -u LD_PRELOAD "$BIN_DIR/hummingbird" --headless --duration 30 >/dev/null 2>&1 &
SIM=$!
sleep 1.5
OUT=$("$BIN_DIR/send_command" --x 1 --y 2 --z 3 --yaw 0.5 --mode 1 2>&1) || fail "send_command could not publish to a live simulator"
echo "$OUT" | grep -q "Published /drone/command: x=1.00 y=2.00 z=3.00 yaw=0.50 mode=1" || fail "send_command did not report the published setpoint"
echo "PASS"
