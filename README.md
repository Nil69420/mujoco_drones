# AeroCore

High-fidelity quadrotor simulation built on [MuJoCo](https://mujoco.org/) with real-time IPC transport and live telemetry visualization.

Simulates a Hummingbird quadrotor with a full sensor suite (IMU, GNSS, barometer, LiDAR, infrared, camera), PD attitude/position flight controller, and zero-copy IPC via [Renoir](https://github.com/Nil69420/renoir). A built-in [Foxglove](https://foxglove.dev/) WebSocket bridge enables real-time visualization from any Foxglove-compatible client.

## Demo

![Hummingbird flight demo](misc/humming.gif)

This media file is large and tracked with Git LFS. If your clone does not pull it automatically:

```bash
git lfs install
git lfs pull
```

## Architecture

Runtime data path:

1. MuJoCo advances rigid-body dynamics from actuator commands.
2. Controller computes rotor thrust and spin commands from state estimates and target setpoint.
3. Sensor subsystem publishes deterministic IMU, GNSS, barometer, LiDAR, infrared, and camera outputs.
4. Renoir transport publishes sensor payloads over shared-memory channels with zero-copy semantics.
5. Foxglove bridge drains transport topics and serves telemetry streams over WebSocket (`ws://0.0.0.0:8765`).

Subsystem boundaries:

| Subsystem | Responsibility | Primary files |
|-----------|----------------|---------------|
| Simulation core | Model loading, stepping loop, CLI integration | `src/main.c`, `model/scene.xml`, `model/hummingbird.xml` |
| Control | Position/attitude control, actuator mapping | `src/controller.c`, `include/controller.h` |
| Sensor synthesis | Signal generation, timing, camera rendering | `src/sensors/sensors.c`, `include/sensors/` |
| IPC transport | Renoir topic publish/subscribe integration | `src/transport/transport_renoir.c`, `include/transport/` |
| Telemetry serving | Foxglove protocol, framing, serialization | `src/foxglove/`, `include/foxglove/` |

## Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| MuJoCo | 3.5.0+ | Installed to `~/.mujoco/mujoco` |
| CMake | 3.10+ | Build system |
| GCC / Clang | C11 | Compiler |
| GLFW3 + OpenGL | — | Optional (headless mode without) |
| Rust | stable | Required for Renoir IPC |
| clang-tidy | — | Static analysis |

### Install MuJoCo

```bash
mkdir -p ~/.mujoco
curl -fsSL https://github.com/google-deepmind/mujoco/releases/download/3.5.0/mujoco-3.5.0-linux-x86_64.tar.gz \
    | tar xz -C ~/.mujoco
mv ~/.mujoco/mujoco-3.5.0 ~/.mujoco/mujoco
```

### Install system dependencies (Ubuntu/Debian)

```bash
sudo apt-get install build-essential cmake libglfw3-dev libgl-dev clang-tidy
```

### Build Renoir

Renoir must be cloned as a sibling directory:

```bash
cd /path/to/parent
git clone git@github.com:Nil69420/renoir.git
cd renoir
cargo build --release --features c-api
```

## Build

```bash
./build.sh                  # Release + IPC + Foxglove
./build.sh debug            # Debug + IPC + Foxglove
./build.sh release OFF      # Release, no IPC
./build.sh lint             # Build + clang-tidy static analysis
```

## Run

```bash
./run.sh all                              # Build + run with Foxglove bridge
./run.sh all --headless --duration 30     # Headless with Foxglove
./run.sh sim                              # Build + run without Foxglove
./run.sh analyze                          # ASan + UBSan dynamic analysis
./run.sh analyze --headless --duration 5  # ASan + UBSan targeted run
./run.sh analyze-tsan --headless --duration 5
```

### Direct execution

```bash
cd build
./hummingbird                             # Interactive viewer
./hummingbird --headless --duration 10    # Headless
./hummingbird --no-ipc                    # No IPC transport
./hummingbird --altitude 2.0             # Custom target altitude
./hummingbird --lidar-rays 72            # Custom LiDAR config
./hummingbird --cam-width 640 --cam-height 360 --cam-fps 8
```

## Dynamic Analysis

AddressSanitizer + UndefinedBehaviorSanitizer:

```bash
./run.sh analyze --headless --duration 10
```

ThreadSanitizer:

```bash
./run.sh analyze-tsan --headless --duration 10
```

Both sanitizer workflows build dedicated debug trees (`build-analyze` and `build-tsan`) to avoid polluting release artifacts.

## Foxglove Visualization

When built with `ENABLE_FOXGLOVE=ON` (default), the simulator starts a WebSocket server on `ws://0.0.0.0:8765`.

Connect with [Foxglove Studio](https://foxglove.dev/):
1. Open Foxglove Studio
2. Open connection → WebSocket → `ws://localhost:8765`
3. Add panels for the published topics

### Published topics

| Topic | Schema | Description |
|-------|--------|-------------|
| `/drone/imu` | `sensor_imu_t` | Accelerometer, gyroscope, magnetometer, orientation |
| `/drone/gnss` | `sensor_gnss_t` | GPS position, velocity, fix quality |
| `/drone/baro` | `sensor_baro_t` | Pressure, temperature, altitude |
| `/drone/lidar` | `sensor_lidar_t` | 2D range scan |
| `/drone/infrared` | `sensor_infrared_t` | Downward-facing range |
| `/drone/camera/meta` | `sensor_camera_meta_t` | Camera resolution, FOV metadata |

### Command uplink

The simulator subscribes to `/drone/command` and applies received setpoints
to the flight controller's target. It is consumed by the simulator, not
published by it — the standalone `send_command` tool publishes a single
setpoint over the Renoir transport:

| Topic | Schema | Direction |
|-------|--------|-----------|
| `/drone/command` | `command_setpoint_t` (x/y/z/yaw as doubles, mode as uint8_t) | Published by `send_command`, consumed by the simulator |

```bash
./send_command --x 1.0 --y 0 --z 1.5 --yaw 0
```

`send_command` is built alongside `hummingbird` when IPC is enabled and
lives in the build directory.

## Project Structure

```
├── include/                  Headers
│   ├── controller.h
│   ├── setpoint.h
│   ├── types.h
│   ├── viewer.h
│   ├── foxglove/             Foxglove bridge headers
│   ├── sensors/              Sensor system headers
│   └── transport/            IPC transport headers
├── src/
│   ├── main.c                Entry point, CLI, sim loop
│   ├── controller.c          PD(I) flight controller
│   ├── viewer.c              GLFW/OpenGL visualization
│   ├── foxglove/             Foxglove WebSocket bridge
│   │   ├── bridge.c          Bridge lifecycle + main thread
│   │   ├── proto.c           Protocol message handling
│   │   ├── serialize.c       JSON serializers
│   │   ├── ws.c              WebSocket framing
│   │   ├── sha1.c            SHA-1 (handshake)
│   │   └── base64.c          Base64 (handshake)
│   ├── sensors/              Sensor simulation + noise
│   └── transport/            Renoir IPC transport
├── model/                    MJCF model files
├── meshes/                   3D mesh assets
├── build.sh                  Build script
├── run.sh                    Run script
├── CMakeLists.txt
└── .github/workflows/ci.yml  CI pipeline
```

## CI/CD

GitHub Actions runs on every push and PR to `master`:

- **Release** — full build with IPC + Foxglove
- **ASan** — AddressSanitizer + UndefinedBehaviorSanitizer
- **TSan** — ThreadSanitizer
- **Static Analysis** — clang-tidy with warnings-as-errors

## License

See repository root for license information.
