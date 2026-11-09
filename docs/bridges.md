# Flight-software bridges

mujoco_drones can replace its built-in controller with a real flight stack running in software-in-the-loop (SITL). Each bridge is lockstep: the flight stack waits for the simulator's reply to every output frame, so simulated time never depends on wall-clock time.

## ArduPilot (JSON backend)

Status: verified against ArduCopter built from ArduPilot master (2026-11-09). A stock copter with default parameters arms, takes off to 5 m, holds within 0.02 m, flies to a waypoint 11 m away with 0.03 m error and lands. Plane, rover, VTOL and sub frames are not mapped yet.

### How it works

The simulator listens on UDP 127.0.0.1:9002. ArduPilot sends a binary servo packet (`magic` 18458 for 16 channels or 29569 for 32, `frame_rate`, `frame_count`, `pwm[]`, little-endian). The simulator applies the PWM values to the rotors, advances physics by one ArduPilot frame (1/`frame_rate` s), and replies with one line of JSON: physics time, gyro and accelerometer in the body frame, position, velocity and attitude in NED. Repeated `frame_count` values are answered with the previous state without advancing physics; a lower `frame_count` (ArduPilot restarted) resets the vehicle and the physics clock.

Reference: ArduPilot `libraries/SITL/SIM_JSON.cpp` and `libraries/SITL/examples/JSON/readme.md`.

### Run it

Build the ArduPilot SITL copter (`./waf configure --board sitl && ./waf copter`), then:

```sh
./build/hummingbird --headless --scenario scenarios/ardupilot_quad.scn
arducopter -S --model JSON:127.0.0.1 --speedup 1 --defaults copter.parm
```

The scenario selects the bridge per vehicle:

```ini
[vehicle ap0]
airframe = quad_x
bridge = ardupilot
bridge_port = 9002
```

`copter.parm` must set `FRAME_CLASS 1` and `FRAME_TYPE 6` (plus frame, reversed propellers): the `quad_x` airframe has its rotors on the axes and its rotor reaction torques have the opposite sign to ArduPilot's plus frame. With `FRAME_TYPE 0` the copter takes off and then tumbles. Set `ARMING_CHECK 0` for the first run if arming is refused.

### What ArduPilot receives

The state is the simulator's ground truth, not the modelled sensors: the JSON backend has no way to feed GNSS or barometer readings, so ArduPilot generates GPS, barometer and compass data internally from the position and attitude. The IMU values come from the simulated accelerometer and gyro without the noise models. Position is relative to where the vehicle is at the first frame, so spawn the vehicle at its resting height.

Mapping from PWM to thrust follows ArduPilot's thrust expo (`MOT_THST_EXPO`, default 0.65), see `ap_map_quad_plus` in `src/bridge/ap_map.c`.

### Tests

- Unit tests: `ap_json` (codec and the JSON parser behaviour ArduPilot uses), `ap_map`, `ap_link` (loopback), `ap_frames` (ENU/FLU to NED/FRD), `ap_runner` (lockstep, duplicates, restart, one-step state lag).
- Fuzz target: `fuzz/fuzz_ap_servo.c` (37 million executions without findings).
- End to end: `tests/ardupilot_e2e.sh` runs the scenario above against a real SITL binary and checks takeoff, waypoint and landing. It skips unless `ARDUPILOT_COPTER`, `ARDUPILOT_COPTER_DEFAULTS` and a Python with `pymavlink` (`ARDUPILOT_PYTHON`) are set.

### Known limits

- Multirotor plus-frame quad only; other airframes need their own mapping tables.
- One bridged vehicle per scenario.
- Sensor models are not yet routed through ArduPilot (see the roadmap: GPS_INPUT).
