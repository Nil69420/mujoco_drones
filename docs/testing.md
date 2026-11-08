# Testing

```bash
cmake -S . -B build -DENABLE_IPC=ON && cmake --build build -j
cd build && ctest --output-on-failure
```

| Test | What it covers |
|------|----------------|
| `test_imu_model`, `test_gnss_model`, `test_misc_models`, `test_camera_model`, `test_conditions` | Statistical behaviour of every model against its stated parameters (Allan deviation, photon-transfer curve, correlation times, geometry). Most were mutation-checked: the test fails when the model is broken. |
| `test_kf_ins` | Estimator accuracy against raw GNSS, uncorrelated noise, latency, outages, no magnetometer. |
| `test_biomes` | Every biome compiles and steps, the compiled terrain matches the height function by ray casting, determinism, spawn clearance, nothing under water. |
| `test_ws_proto`, `test_fg_ops`, `fuzz/fuzz_ws_proto.c` | WebSocket handshake, frame and message parsing; libFuzzer target. |
| `tests/foxglove_adversarial.py` | 27 hostile-client scenarios against a live simulator (run under ASan and TSan; see `docs/foxglove.md`). |
| `test_cli`, `test_import_config`, `importer_golden` | Option parsing, importer config parsing, and golden output of the URDF importer. |
| `sensor_profiles_live`, `wind_live` | Live simulator: exact published rates, vibration, magnetometer, rangefinder, 1280x720 at 120 fps mono8 through the frame ring, wind pushing a hovering drone. |

Static analysis: `cmake --build build --target lint` (clang-tidy with warnings as errors) covers every
source. Sanitizers: configure with `-DENABLE_ASAN=ON` or `-DENABLE_TSAN=ON`. Use `tests/lsan.supp` to ignore
leaks inside the GPU driver's `libdbus`, and `tests/tsan.supp` plus `setarch x86_64 -R` for TSan on kernels
with high-entropy ASLR.

Live tests need the Renoir shared memory directory (`RENOIR_SHM_DIR`, set by the scripts) and, for the camera
part, an EGL device. Where no EGL device exists the camera test cannot run.
