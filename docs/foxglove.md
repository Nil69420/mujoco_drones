# Foxglove WebSocket bridge

Build with `-DENABLE_FOXGLOVE=ON`. The bridge serves the simulator's sensor channels over the Foxglove WebSocket protocol (`foxglove.websocket.v1`, JSON encoding) on port 8765.

## Security model

- It listens on **127.0.0.1 only**. To reach it from another machine set `MUJOCO_DRONES_FOXGLOVE_BIND` to an interface address (for example `0.0.0.0`). The bridge has no authentication and no TLS; expose it only on a trusted network or through an SSH tunnel.
- Clients can only read channels and manage their own subscriptions. There is no command path through the bridge.

## Behaviour and limits

- Single bridge thread, fully non-blocking: a slow, silent or hostile client cannot stall the others.
- Up to 4 clients; further connections receive `503 Service Unavailable`. Up to 8 half-open handshakes are tracked and dropped after 3 seconds.
- Each client has a 4 MiB output queue. When it is full, new messages for that client are dropped (and counted) instead of blocking; a client that makes no progress for 10 seconds is disconnected. TCP keepalive removes dead peers.
- Client messages are limited to 64 KiB and are parsed with a strict JSON parser (depth limit 16). Subscribing to an unknown channel, reusing a subscription id or exceeding 32 subscriptions returns a `status` message with level 2 and leaves the connection open.
- RFC 6455 is enforced: case-insensitive headers, `GET` with `Upgrade`, `Connection` and `Sec-WebSocket-Version: 13`, a valid key, masked client frames, no reserved bits, fragmentation and control-frame rules, valid UTF-8, and a close handshake. Violations close the connection with the proper status code.
- The bridge forwards only the latest sample of each channel per loop iteration, so the stream is for live viewing. It is not a lossless recording path; use the Renoir topics directly to record every sample.

## Testing

- `build/test_ws_proto` and `build/test_fg_ops` (also run by `ctest`): protocol and message-parser unit tests.
- `fuzz/fuzz_ws_proto.c`: libFuzzer target for the handshake, frame parser and message parser (`clang -fsanitize=fuzzer,address,undefined`).
- `tests/foxglove_adversarial.py <path to hummingbird>`: 27 attack scenarios against a live simulator (idle and dribbling connections, malformed handshakes, protocol violations, hostile JSON, stalled readers, connection limit). Run it against ASan and TSan builds; TSan needs `SIM_NO_ASLR=1 SIM_LD_PRELOAD=$(cc -print-file-name=libtsan.so) TSAN_OPTIONS=suppressions=tests/tsan.supp` on this kernel.
