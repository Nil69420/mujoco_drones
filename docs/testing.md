# Testing

```bash
cmake -S . -B build -DENABLE_IPC=ON && cmake --build build -j
cd build && ctest --output-on-failure
```

| Test | What it covers |
|------|----------------|
| `test_imu_model`, `test_gnss_model`, `test_misc_models`, `test_camera_model`, `test_conditions` | Statistical behaviour of every model against its stated parameters (Allan deviation, photon-transfer curve, correlation times, geometry). Most were mutation-checked: the test fails when the model is broken. |
| `test_airframe_effects`, `test_scenario_effects`, `test_cli` | The four airframe effects (validation, thrust-scaled deficit, first-order lag, drop rate, timestamp shift), their all-or-none scenario keys and options. |
| `test_kf_ins` | Estimator accuracy against raw GNSS, uncorrelated noise, latency, outages, no magnetometer. |
| `test_biomes` | Every biome compiles and steps, the compiled terrain matches the height function by ray casting, determinism, spawn clearance, nothing under water. |
| `test_ws_proto`, `test_fg_ops`, `fuzz/fuzz_ws_proto.c` | WebSocket handshake, frame and message parsing; libFuzzer target. |
| `tests/foxglove_adversarial.py` | 27 hostile-client scenarios against a live simulator (run under ASan and TSan; see the Foxglove page). |
| `test_import_pose`, `test_import_config`, `importer_golden`, `importer_sdf_golden` | Rigid-pose maths, importer config parsing, golden output of the URDF importer, and the SDF importer: pose composition (hand-derived expected origins), model-frame axes, SDF directory discovery and its error paths. |
| `test_ap_*`, `test_px4_*`, `test_bf_*`, `test_mav2` | Codecs, links, lockstep runners and frame conversions for the three flight-stack bridges and the MAVLink v2 core. |
| `scenario_smoke_*` | Every bridgeless scenario runs headless for five simulated seconds, exits cleanly and prints finite final states. |
| `tools_smoke` | `pilot_tool` and `send_command`: usage, argument validation, missing devices, and publishing a setpoint to a live simulator. |
| `sensor_profiles_live`, `wind_live`, `multirotor_live` | Live simulator: exact published rates, vibration measured after a fixed simulated time, magnetometer, rangefinder, 1280x720 at 120 fps mono8 through the frame ring, thrust deficit applied at hover, wind pushing a hovering drone. |
| `ardupilot_e2e`, `ardupilot_effects_e2e` | Stock ArduCopter 4.6.1: arm, take off, hold, waypoint, land; then the same flight with and without a thrust deficit, checking that the IMU shifts by 15 to 45 mG and the hold altitude by 0.1 to 1 m. |
| `px4_e2e` | PX4 v1.17.0: arm, take off to 5 m, waypoint, land and disarm. |
| `betaflight_e2e`, `betaflight_sticks_e2e` | Betaflight master: boot, configure, arm, lift off, hold 3 m, come back down. The stick-response test is experimental and does not gate (see the bridges page). |

## End-to-end tests need a flight stack

Each end-to-end test skips unless its binary is configured. The versions CI uses are pinned by commit:

| Stack | Variables | Version and build |
|-------|-----------|-------------------|
| ArduCopter | `ARDUPILOT_COPTER`, `ARDUPILOT_COPTER_DEFAULTS`, `ARDUPILOT_PYTHON` | `Copter-4.6.1` (`57591cb8`): `./waf configure --board sitl && ./waf build --target bin/arducopter`; `defaults` is `Tools/autotest/default_params/copter.parm`; the Python needs `pymavlink` |
| PX4 | `PX4_BIN`, `PX4_ETC`, `PX4_PYTHON` | `v1.17.0` (`d6f12ad1`): copy `boards/px4/sitl/default.px4board` to `nogz.px4board` with the three `GZ_*` modules disabled, then `make px4_sitl_nogz` |
| Betaflight | `BETAFLIGHT_SITL` | commit `6241cfb1`: initialise the `mavlink`, `dronecan/libcanard` and `src/config` submodules one at a time, then `make TARGET=SITL` |

## Continuous integration

Every job runs on Ubuntu 24.04 with every action pinned by commit, so a runner-image or toolchain change cannot silently move the gate. The workflows are `ci.yml` (Release, ASan, TSan, clang-tidy, fuzzing, the Foxglove adversarial suite) and `verify.yml`:

- **Line coverage gate**: gcov instrumentation of every target, all tests including the three flight-stack end-to-end tests, and a failure below **80 percent of lines** (`COVERAGE_MIN_LINES` in `verify.yml`; the viewer, which needs a display, is excluded).
- **cppcheck** over the whole source tree, and **valgrind** over every unit-test binary with a suppression limited to MuJoCo's own zero-size `aligned_alloc` in `mj_setConst`.
- **ArduCopter 4.6.1, PX4 v1.17.0 and Betaflight end to end** in Release; the flight stacks are built once and cached by commit and runner image.
- A nightly **canary** on the newest runner image runs the linters and does not block anything, so toolchain drift shows up before it turns the gate red.

Static analysis locally: `cmake --build build --target lint` (clang-tidy with warnings as errors; CI uses clang-tidy 18). Sanitizers: `-DENABLE_ASAN=ON` or `-DENABLE_TSAN=ON`; every target is instrumented automatically. Use `tests/lsan.supp` for leaks inside the GPU driver's `libdbus`, and `tests/tsan.supp` plus `setarch x86_64 -R` for TSan on kernels with high-entropy ASLR. Coverage: `-DENABLE_COVERAGE=ON`, run `ctest`, then `gcovr --root . --filter src/`.

Live tests need the Renoir shared memory directory (`RENOIR_SHM_DIR`, set by the scripts) and, for the camera part, an EGL device. Where no EGL device exists the camera test cannot run. On a software renderer the camera delivers only a few frames, so its rate is checked within 5 percent instead of exactly.
