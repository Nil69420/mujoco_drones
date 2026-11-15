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
                yaw_deg = float(line.split("yaw=")[1])
                rows.append((time_s, float(numbers[0]), float(numbers[1]), float(numbers[2]), yaw_deg))
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


def pulse(radio, index, value, seconds=1.4, settle=2.5):
    rows = history()
    start = rows[-1]
    before = sample_at(rows, start[0] - 1.0)
    span = max(start[0] - before[0], 0.25)
    velocity = ((start[1] - before[1]) / span, (start[2] - before[2]) / span)
    radio.channels[index] = value
    time.sleep(seconds)
    radio.channels[index] = 1500
    time.sleep(settle)
    end = history()[-1]
    elapsed = end[0] - start[0]
    east = end[1] - start[1] - velocity[0] * elapsed
    north = end[2] - start[2] - velocity[1] * elapsed
    heading = math.radians(start[4])
    forward = east * math.cos(heading) + north * math.sin(heading)
    right = east * math.sin(heading) - north * math.cos(heading)
    return forward, right


def lift_off(radio, ground):
    for _ in range(4):
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


def main():
    global SIM_LOG
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
    radio.channels[2] = HOVER_THROTTLE
    time.sleep(1.5)
    print("altitude %.2f m above the ground" % (truth()[2] - ground))
    forward = pulse(radio, 1, 1700)
    back = pulse(radio, 1, 1300)
    right = pulse(radio, 0, 1700)
    left = pulse(radio, 0, 1300)
    print("pitch forward %s back %s roll right %s left %s (forward, right metres)" % (forward, back, right, left))
    if None in (forward, back, right, left):
        print("FAIL: lost telemetry")
        return 1
    if forward[0] - back[0] < 0.5:
        print("FAIL: pitch stick direction")
        return 1
    if right[1] - left[1] < 0.3:
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
    radio.channels[2] = 1000
    time.sleep(3)
    radio.channels[4] = 1000
    radio.running = False
    print("PASS")
    return 0


sys.exit(main())
