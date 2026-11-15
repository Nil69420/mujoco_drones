#!/usr/bin/env python3
import asyncio
import base64
import json
import os
import socket
import struct
import subprocess
import sys
import time

import websockets

PORT = 8765
HOST = "127.0.0.1"
KEY = base64.b64encode(os.urandom(16)).decode()
HANDSHAKE = (
    "GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
    f"Sec-WebSocket-Key: {KEY}\r\nSec-WebSocket-Version: 13\r\n"
    "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n"
)
results = []


def record(name, ok, detail=""):
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name + (f" - {detail}" if detail else ""), flush=True)


def connect(timeout=3.0):
    s = socket.create_connection((HOST, PORT), timeout=timeout)
    s.settimeout(timeout)
    return s


def upgrade(s):
    s.sendall(HANDSHAKE.encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            raise ConnectionError("closed during handshake")
        buf += chunk
    if b" 101 " not in buf.split(b"\r\n")[0]:
        raise ConnectionError(buf.split(b"\r\n")[0].decode(errors="replace"))
    return buf.split(b"\r\n\r\n", 1)[1]


def masked_frame(opcode, payload, fin=True, rsv=0, mask=True):
    b0 = (0x80 if fin else 0) | rsv | opcode
    m = os.urandom(4)
    n = len(payload)
    head = bytes([b0])
    flag = 0x80 if mask else 0
    if n < 126:
        head += bytes([flag | n])
    elif n < 65536:
        head += bytes([flag | 126]) + struct.pack(">H", n)
    else:
        head += bytes([flag | 127]) + struct.pack(">Q", n)
    if mask:
        return head + m + bytes(p ^ m[i & 3] for i, p in enumerate(payload))
    return head + payload


def drain_until_closed(s, limit=3.0):
    end = time.time() + limit
    data = b""
    try:
        while time.time() < end:
            chunk = s.recv(65536)
            if not chunk:
                return True, data
            data += chunk
    except socket.timeout:
        return False, data
    except OSError:
        return True, data
    return False, data


async def baseline(label, seconds=1.5):
    try:
        async with websockets.connect(f"ws://{HOST}:{PORT}", subprotocols=["foxglove.websocket.v1"],
                                      open_timeout=4) as ws:
            await asyncio.wait_for(ws.recv(), 3)
            adv = json.loads(await asyncio.wait_for(ws.recv(), 3))
            subs = [{"id": i, "channelId": c["id"]} for i, c in enumerate(adv["channels"])]
            await ws.send(json.dumps({"op": "subscribe", "subscriptions": subs}))
            n = 0
            end = time.time() + seconds
            while time.time() < end:
                try:
                    m = await asyncio.wait_for(ws.recv(), 1.0)
                    n += isinstance(m, bytes)
                except asyncio.TimeoutError:
                    break
            return n
    except Exception as e:
        print(f"   baseline error in {label}: {type(e).__name__}: {e}")
        return -1


async def check(name, attack, min_messages=50):
    try:
        attack()
    except Exception as e:
        record(name, False, f"attack harness error {type(e).__name__}: {e}")
        return
    n = await baseline(name)
    record(name, n >= min_messages, f"{n} messages for a normal client afterwards")


def attack_many_idle():
    socks = []
    for _ in range(40):
        try:
            socks.append(connect(1.0))
        except OSError:
            pass
    time.sleep(0.5)
    for s in socks:
        s.close()


def attack_slowloris():
    s = connect()
    for ch in b"GET / HTT":
        s.send(bytes([ch]))
        time.sleep(0.1)
    closed, _ = drain_until_closed(s, limit=5.0)
    if not closed:
        raise RuntimeError("server kept a half-open handshake for more than 5 s")
    s.close()


def make_bad_handshake(payload):
    def run():
        s = connect()
        s.sendall(payload)
        drain_until_closed(s, limit=2.0)
        s.close()
    return run


def make_frame_attack(frame, expect_close=True):
    def run():
        s = connect()
        upgrade(s)
        s.sendall(frame)
        closed, _ = drain_until_closed(s, limit=2.5)
        if expect_close and not closed:
            raise RuntimeError("server did not close after a protocol violation")
        s.close()
    return run


def make_message_attack(messages):
    def run():
        s = connect()
        upgrade(s)
        for m in messages:
            s.sendall(masked_frame(1, m if isinstance(m, bytes) else m.encode()))
        time.sleep(0.3)
        s.close()
    return run


def attack_slow_reader():
    s = connect()
    leftover = upgrade(s)
    del leftover
    sub = {"op": "subscribe", "subscriptions": [{"id": i, "channelId": i + 1} for i in range(7)]}
    s.sendall(masked_frame(1, json.dumps(sub).encode()))
    time.sleep(14.0)
    closed, _ = drain_until_closed(s, limit=2.0)
    s.close()


def attack_mid_frame_reset():
    s = connect()
    upgrade(s)
    s.sendall(masked_frame(1, b"x" * 100)[:30])
    s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
    s.close()


def attack_clean_close():
    s = connect()
    upgrade(s)
    s.sendall(masked_frame(8, struct.pack(">H", 1000) + b"bye"))
    closed, data = drain_until_closed(s, limit=2.0)
    if not (data[:1] == b"\x88" or closed):
        raise RuntimeError("no close reply")
    s.close()


def attack_extra_clients():
    socks = []
    rejected = False
    for _ in range(8):
        s = connect()
        try:
            upgrade(s)
            socks.append(s)
        except ConnectionError as e:
            rejected = "503" in str(e)
            s.close()
            break
    for s in socks:
        s.close()
    if not rejected or len(socks) != 4:
        raise RuntimeError(f"expected 4 clients then 503, got {len(socks)} clients, rejected={rejected}")


async def main(binary):
    env = dict(os.environ)
    env.setdefault("RENOIR_SHM_DIR", "/tmp")
    env.pop("LD_PRELOAD", None)
    if os.environ.get("SIM_LD_PRELOAD"):
        env["LD_PRELOAD"] = os.environ["SIM_LD_PRELOAD"]
    cmd = [binary, "--headless", "--duration", "10000000"]
    if os.environ.get("SIM_NO_ASLR"):
        preload = env.pop("LD_PRELOAD", None)
        cmd = ["setarch", "x86_64", "-R"] + (["env", f"LD_PRELOAD={preload}"] if preload else []) + cmd
    proc = subprocess.Popen(cmd, cwd=os.path.dirname(binary),
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, env=env)
    for _ in range(300):
        try:
            socket.create_connection((HOST, PORT), timeout=0.3).close()
            break
        except OSError:
            time.sleep(0.1)
    else:
        print("server never started")
        return 2

    base = await baseline("startup", 2.0)
    record("baseline", base > 100, f"{base} messages in 2 s")

    await check("40 idle connections", attack_many_idle)
    await check("slowloris handshake is dropped", attack_slowloris)
    await check("garbage bytes", make_bad_handshake(os.urandom(2000)))
    await check("plain HTTP GET", make_bad_handshake(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n"))
    await check("POST request", make_bad_handshake(b"POST / HTTP/1.1\r\nHost: x\r\n\r\n"))
    await check("64 KiB header", make_bad_handshake(b"GET / HTTP/1.1\r\nX: " + b"A" * 65536 + b"\r\n\r\n"))
    await check("lowercase header names are accepted", lambda: _lower_ok())
    await check("unmasked client frame", make_frame_attack(masked_frame(1, b"{}", mask=False)))
    await check("reserved bits set", make_frame_attack(masked_frame(1, b"{}", rsv=0x40)))
    await check("reserved opcode", make_frame_attack(masked_frame(3, b"{}")))
    await check("2^63 byte frame", make_frame_attack(bytes([0x81, 0xFF]) + b"\x80" + b"\0" * 7 + os.urandom(4)))
    await check("1 GiB frame", make_frame_attack(bytes([0x81, 0xFF]) + struct.pack(">Q", 1 << 30) + os.urandom(4)))
    await check("fragmented ping", make_frame_attack(masked_frame(9, b"x", fin=False)))
    await check("continuation without start", make_frame_attack(masked_frame(0, b"x")))
    await check("invalid UTF-8 text", make_frame_attack(masked_frame(1, b"\xc0\x80")))
    await check("invalid JSON", make_message_attack(["{not json", "[", "null", "{\"op\":", "\x00\x01"]), 50)
    await check("deeply nested JSON", make_message_attack(["[" * 5000 + "]" * 5000, "{\"a\":" * 2000 + "1" + "}" * 2000]))
    await check("bad subscriptions", make_message_attack([
        json.dumps({"op": "subscribe", "subscriptions": [{"id": 1, "channelId": 0}, {"id": 2, "channelId": 99999},
                                                         {"id": 3, "channelId": 1}, {"id": 3, "channelId": 2},
                                                         {"id": -1, "channelId": 1}, {"id": 1.5, "channelId": 1}]}),
        json.dumps({"op": "subscribe", "subscriptions": [{"id": i, "channelId": 1} for i in range(500)]}),
        json.dumps({"op": "unsubscribe", "subscriptionIds": list(range(500))}),
    ]))
    await check("oversized text message", make_message_attack(["{\"op\":\"subscribe\",\"x\":\"" + "A" * 70000 + "\"}"]))
    await check("reset in the middle of a frame", attack_mid_frame_reset)
    await check("clean close handshake", attack_clean_close)
    await check("clients beyond the limit", attack_extra_clients)
    rss_before = _rss(proc.pid)
    await check("client that never reads", attack_slow_reader)
    rss_after = _rss(proc.pid)
    record("memory stays bounded with a stalled reader", rss_after - rss_before < 100_000, f"{rss_after - rss_before} kB growth")

    alive = proc.poll() is None
    record("simulator still running", alive)
    proc.terminate()
    try:
        err = proc.communicate(timeout=15)[1].decode(errors="replace")
    except subprocess.TimeoutExpired:
        proc.kill()
        err = ""
    bad = [l for l in err.splitlines() if "Sanitizer" in l or "runtime error" in l]
    record("no sanitizer reports", not bad, "; ".join(bad[:3]))
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)} passed, {len(failed)} failed")
    return 1 if failed else 0


def _lower_ok():
    s = connect()
    req = HANDSHAKE.replace("Upgrade:", "upgrade:").replace("Connection:", "connection:") \
                   .replace("Sec-WebSocket-Key", "sec-websocket-key").replace("Sec-WebSocket-Version", "SEC-WEBSOCKET-VERSION")
    s.sendall(req.encode())
    reply = s.recv(512)
    s.close()
    if b" 101 " not in reply.split(b"\r\n")[0]:
        raise RuntimeError(f"lowercase headers rejected: {reply[:60]!r}")


def _rss(pid):
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: foxglove_adversarial.py /path/to/hummingbird (built with ENABLE_FOXGLOVE=ON)")
        sys.exit(2)
    sys.exit(asyncio.run(main(os.path.abspath(sys.argv[1]))))
