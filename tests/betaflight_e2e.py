import math
import socket
import struct
import sys
import threading
import time

MSP_PORT = 5761
RC_PORT = 9004
HOVER_THROTTLE = 1262
CLIMB_THROTTLE = 1330
EARTH_M_PER_DEG = 111320.0


def cli(commands, pause=0.4):
    sock = socket.create_connection(("127.0.0.1", MSP_PORT), timeout=5)
    sock.settimeout(0.5)

    def drain():
        data = b""
        try:
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
        except socket.timeout:
            pass
        return data

    sock.sendall(b"#\n")
    time.sleep(0.5)
    drain()
    for command in commands:
        sock.sendall((command + "\n").encode())
        time.sleep(pause)
        drain()
    sock.close()


def msp(command):
    sock = socket.create_connection(("127.0.0.1", MSP_PORT), timeout=3)
    sock.settimeout(1.0)
    body = bytes([0, command])
    checksum = 0
    for value in body:
        checksum ^= value
    sock.sendall(b"$M<" + body + bytes([checksum]))
    data = b""
    end = time.time() + 1.0
    while time.time() < end:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        data += chunk
        if len(data) >= 6 and len(data) >= 5 + data[3] + 1:
            break
    sock.close()
    start = data.find(b"$M>")
    if start < 0:
        return None
    return data[start + 5:start + 5 + data[start + 3]]


def retry(command, minimum):
    for _ in range(10):
        reply = msp(command)
        if reply is not None and len(reply) >= minimum:
            return reply
        time.sleep(0.2)
    return None


SIM_LOG = ""


def history():
    rows = []
    with open(SIM_LOG) as handle:
        for line in handle:
            if line.startswith("[bridge] t="):
                time_s = float(line.split("t=")[1].split()[0])
                numbers = line.split("pos=(")[1].split(")")[0].split()
                yaw_deg = float(line.split("yaw=")[1].split()[0])
                pitch_up = float(line.split("pitch_up=")[1].split()[0])
                roll_right = float(line.split("roll_right=")[1].split()[0])
                rows.append((time_s, float(numbers[0]), float(numbers[1]), float(numbers[2]), yaw_deg, pitch_up, roll_right))
    return rows


def truth():
    rows = history()
    return rows[-1][1:] if rows else None


def sample_at(rows, time_s):
    best = rows[0]
    for row in rows:
        if abs(row[0] - time_s) < abs(best[0] - time_s):
            best = row
    return best


class Radio:
    def __init__(self):
        self.channels = [1500, 1500, 1000, 1500, 1000, 1000] + [1500] * 10
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.start = time.time()
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while self.running:
            packet = struct.pack("<d16H", time.time() - self.start, *self.channels)
            self.sock.sendto(packet, ("127.0.0.1", RC_PORT))
            time.sleep(0.01)


ATTITUDE_COLUMN = {"pitch_up": 5, "roll_right": 6}
RESPONSE_DEGREES = 6.0
SETTLE_SECONDS = 2.5


def pulse(radio, index, value, axis, seconds=1.2):
    time.sleep(SETTLE_SECONDS)
    column = ATTITUDE_COLUMN[axis]
    rows = history()
    if not rows:
        return None
    start_time = rows[-1][0]
    baseline = rows[-1][column]
    radio.channels[index] = value
    time.sleep(seconds)
    radio.channels[index] = 1500
    window = [row[column] - baseline for row in history() if row[0] >= start_time]
    if not window:
        return None
    return max(window, key=abs)


class AltitudeHold:
    TARGET_ABOVE_GROUND = 3.0
    MIN_THROTTLE = 1100
    MAX_THROTTLE = 1500
    PROPORTIONAL = 70.0
    DERIVATIVE = 55.0

    def __init__(self, radio, ground):
        self.radio = radio
        self.target = ground + self.TARGET_ABOVE_GROUND
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        previous = None
        while self.running:
            rows = history()
            if len(rows) >= 2:
                now, before = rows[-1], rows[-2]
                span = now[0] - before[0]
                velocity = (now[3] - before[3]) / span if span > 1e-6 else 0.0
                error = self.target - now[3]
                command = HOVER_THROTTLE + self.PROPORTIONAL * error - self.DERIVATIVE * velocity
                self.radio.channels[2] = int(min(max(command, self.MIN_THROTTLE), self.MAX_THROTTLE))
                previous = now
            time.sleep(0.1)

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)


def lift_off(radio, ground):
    for _ in range(12):
        radio.channels[4] = 2000
        radio.channels[5] = 2000
        time.sleep(1.5)
        radio.channels[2] = CLIMB_THROTTLE
        time.sleep(2.5)
        state = truth()
        if state is not None and state[2] - ground > 0.3:
            return True
        radio.channels[2] = 1000
        radio.channels[4] = 1000
        time.sleep(2.0)
    return False


def finish_flight(radio, hold, ground):
    samples = []
    end = time.time() + 8.0
    while time.time() < end:
        samples.append(truth()[2] - ground)
        time.sleep(0.25)
    low, high = min(samples), max(samples)
    print("altitude held between %.2f and %.2f m above the ground" % (low, high))
    if low < AltitudeHold.TARGET_ABOVE_GROUND - 1.0 or high > AltitudeHold.TARGET_ABOVE_GROUND + 1.0:
        print("FAIL: altitude hold")
        return 1
    hold.stop()
    radio.channels[2] = 1000
    time.sleep(6.0)
    touchdown = truth()[2] - ground
    print("altitude after cutting the throttle %.2f m" % touchdown)
    if touchdown > 0.5:
        print("FAIL: did not come back down")
        return 1
    radio.channels[4] = 1000
    radio.running = False
    print("PASS")
    return 0


def main():
    global SIM_LOG
    mode = sys.argv[3] if len(sys.argv) > 3 else "flight"
    if len(sys.argv) > 2 and sys.argv[1] == "fly":
        SIM_LOG = sys.argv[2]
    if len(sys.argv) > 1 and sys.argv[1] == "configure":
        with open(sys.argv[2]) as handle:
            cli([line.strip() for line in handle if line.strip()])
        return 0
    radio = Radio()
    time.sleep(8)
    ground = truth()[2]
    if not lift_off(radio, ground):
        print("FAIL: did not lift off")
        return 1
    hold = AltitudeHold(radio, ground)
    deadline = time.time() + 25.0
    while time.time() < deadline and abs(truth()[2] - hold.target) > 0.4:
        time.sleep(0.5)
    time.sleep(2.0)
    height = truth()[2] - ground
    print("altitude %.2f m above the ground" % height)
    if abs(height - AltitudeHold.TARGET_ABOVE_GROUND) > 1.0:
        print("FAIL: could not hold %.0f m" % AltitudeHold.TARGET_ABOVE_GROUND)
        return 1
    if mode == "flight":
        return finish_flight(radio, hold, ground)
    forward = pulse(radio, 1, 1700, "pitch_up")
    back = pulse(radio, 1, 1300, "pitch_up")
    right = pulse(radio, 0, 1700, "roll_right")
    left = pulse(radio, 0, 1300, "roll_right")
    print("attitude change: forward stick pitch_up %s, back stick pitch_up %s, right stick roll_right %s, left stick roll_right %s (degrees)" % (forward, back, right, left))
    if None in (forward, back, right, left):
        print("FAIL: telemetry was lost")
        return 1
    if not (forward <= -RESPONSE_DEGREES and back >= RESPONSE_DEGREES):
        print("FAIL: pitch stick direction")
        return 1
    if not (right >= RESPONSE_DEGREES and left <= -RESPONSE_DEGREES):
        print("FAIL: roll stick direction")
        return 1
    before = truth()[3]
    radio.channels[3] = 1650
    time.sleep(3.0)
    radio.channels[3] = 1500
    time.sleep(1.0)
    after = truth()[3]
    turned = (before - after) % 360
    print("yaw turned %d degrees" % turned)
    if not 25 <= turned <= 270:
        print("FAIL: yaw stick direction")
        return 1
    hold.stop()
    radio.channels[2] = 1000
    time.sleep(3)
    radio.channels[4] = 1000
    radio.running = False
    print("PASS")
    return 0


sys.exit(main())
