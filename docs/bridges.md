# Flight-software bridges

mujoco_drones can replace its built-in controller with a real flight stack running in software-in-the-loop (SITL). Each bridge is lockstep: the flight stack waits for the simulator's reply to every output frame, so simulated time never depends on wall-clock time. A scenario selects the bridge per vehicle with `bridge = ardupilot` or `bridge = px4`; `bridge_port` overrides the port and `bridge_speed` sets the simulation speed against wall-clock time (default 1.0, real time).

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
- PX4 receives ground-truth GPS; the GNSS model is not routed yet.
- Sensor models are not yet routed through ArduPilot (see the roadmap: GPS_INPUT).

## PX4 (simulator MAVLink API)

Status: verified against PX4 built from main (2026-11-09) without Gazebo. A multicopter with default parameters arms, takes off to 5 m (4.99 m measured), flies to a waypoint 11 m away with 0.04 m error and lands and disarms. Fixed-wing and VTOL airframes are not mapped yet.

### How it works

PX4's `simulator_mavlink` module connects to the simulator as a TCP client on port 4560, so the simulator listens. Messages are MAVLink v2. The simulator sends `HIL_SENSOR` every 4 ms of simulation time (250 Hz) and `HIL_GPS` at 10 Hz; PX4 answers each `HIL_SENSOR` with `HIL_ACTUATOR_CONTROLS` (motor commands as fractions of full thrust) once its estimator and controllers have run. Until the first reply the simulator free-runs so PX4's clock starts, then it waits for every reply. If PX4 closes the connection the run stops with a message.

The MAVLink codec is hand-written (`src/bridge/mav2.c`, `px4_msgs.c`) and checked byte for byte against frames produced by the MAVLink C library generated for PX4 (`tests/test_mav2.c`), and fuzzed (`fuzz/fuzz_mav2.c`).

### Run it

Build PX4 without the Gazebo modules (they need a matching protobuf), for example a board file that sets `CONFIG_MODULES_SIMULATION_GZ_MSGS=n`, `..._GZ_BRIDGE=n`, `..._GZ_PLUGINS=n`, then:

```sh
./build/hummingbird --headless --scenario scenarios/px4_quad.scn
cd rootfs && cp /path/to/mujoco_drones/tests/px4/10099_mujoco_plus_quad etc/init.d-posix/airframes/
PX4_SIM_MODEL=mujoco_plus_quad px4 -d etc
```

`tests/px4/10099_mujoco_plus_quad` describes the simulator's plus-layout quad to PX4's control allocator (rotor positions and torque signs); it is a copy-in airframe, nothing in PX4 needs patching. Connect a ground station or script to UDP 14550.

### What PX4 receives

Accelerometer, gyro, magnetometer and barometer values come from the modelled sensors when a profile is selected in the scenario (`imu`, `mag`, `baro`), converted to PX4's frame and units, and each sensor is flagged as updated only when it has a new sample. Without model profiles the values are derived from ground truth. The model noise matters: PX4 declares a sensor stuck (`STALE`) when it reports exactly the same value 100 times in a row, so a noise-free magnetometer or barometer at rest is rejected and arming fails with "no valid barometer data". GPS currently comes from ground truth (position, velocity), at 10 Hz.

### Tests

`px4_sensors` (ISA pressure, geomagnetic rotation, geodetic conversion, model overlay), `px4_link` (TCP, partial frames, peer closing, no SIGPIPE), `px4_runner` (startup free-run, lockstep with a fake PX4, no re-send while waiting, connection loss), `mav2`. End to end: `tests/px4_e2e.sh` (skips unless `PX4_BIN` points at a PX4 SITL binary and `PX4_PYTHON` has `pymavlink`).

## Betaflight (SITL UDP)

Status: experimental. Stock Betaflight SITL (built with `make TARGET=SITL`, default Gazebo-bridge conventions) receives the simulator's state, runs its real attitude estimator on the modelled gyro and accelerometer, arms from UDP RC, flies, and answers stick inputs in the right directions in single runs. The closed-loop test `tests/betaflight_e2e.sh` is not a reliable gate yet: Betaflight has no altitude hold, so open-loop stick pulses drift widely and the pass rate over three consecutive runs was below 100%. Do not rely on it as a regression gate.

### How it works

Betaflight sends four motor fractions (UDP, 16 bytes) to port 9002 and listens for the state packet (144 bytes) on 9003 and RC (timestamp and 16 channels, 40 bytes) on 9004. The simulator binds 9002 and sends state to `bridge_port + 1`. Betaflight scales its own clock by the speed it measures from consecutive state timestamps, so the simulator must run at real time (`bridge_speed` 1.0) and must never resend a packet with an unchanged timestamp: a zero speed freezes Betaflight's clock permanently. After a gap of more than 500 ms Betaflight resynchronises, so the runner only retries after 700 ms without a reply.

The Gazebo-bridge variant of the packet is reproduced: gyro and accelerometer in the FRD frame, the attitude quaternion conjugated by a rotation of pi about x, position as longitude, latitude and altitude, with latitude and longitude mirrored about the first position (Betaflight undoes that mirroring).

### Configure

`tests/betaflight/config.cli` gives Betaflight a custom plus-layout mixer matching the simulator's rotor order (with yaw signs checked by flying: the opposite signs make Betaflight's runaway protection disarm), arm on AUX1 and angle mode on AUX2. Apply it over the CLI on TCP port 5761 and `save`; the SITL process exits and must be restarted.

### Hover

Betaflight's motor output is linear in throttle ((throttle - 1000) / 1000, idle 5.5%). For the 0.68 kg quad hover is near 1260 to 1285.

### Known limits

- Betaflight's reported GPS and altitude come from the packet (virtual GPS); magnetometer fusion is off by default.
- Single bridged vehicle, quad plus layout only.
- No altitude hold in Betaflight: validation by stick direction needs closed-loop altitude control from the pilot or a pilot-input layer (see the roadmap, workstream C7).
