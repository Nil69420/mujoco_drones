# mujoco_drones

High-fidelity quadrotor simulation built on [MuJoCo](https://mujoco.org/) with real-time IPC transport and live telemetry visualization.

Simulates a Hummingbird quadrotor with a full sensor suite (IMU, GNSS, barometer, LiDAR, infrared, camera), PD attitude/position flight controller, and zero-copy IPC via [Renoir](https://github.com/Nil69420/renoir). A built-in [Foxglove](https://foxglove.dev/) WebSocket bridge enables real-time visualization from any Foxglove-compatible client.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  MuJoCo Physics Engine                                  │
│  (hummingbird.xml + scene.xml)                          │
├──────────┬──────────┬───────────────────────────────────┤
│ Viewer   │Controller│ Sensors                           │
│ (GLFW/GL)│ PD(I)    │ IMU · GNSS · Baro · LiDAR · IR · │
│          │          │ Camera                            │
├──────────┴──────────┴───────────────────────────────────┤
│  Renoir IPC Transport (zero-copy pub/sub)               │
├─────────────────────────────────────────────────────────┤
│  Foxglove WebSocket Bridge (ws://0.0.0.0:8765)          │
└─────────────────────────────────────────────────────────┘
```

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
./run.sh analyze 5                        # Dynamic analysis, 5s duration
```

### Direct execution

```bash
cd build
./hummingbird                             # Interactive viewer
./hummingbird --headless --duration 10    # Headless
./hummingbird --no-ipc                    # No IPC transport
./hummingbird --altitude 2.0             # Custom target altitude
./hummingbird --lidar-rays 72            # Custom LiDAR config
```

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
