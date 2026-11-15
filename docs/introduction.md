# mujoco_drones

High-fidelity multirotor and fixed-wing simulation built on [MuJoCo](https://mujoco.org/), with zero-copy sensor streaming over [Renoir](https://nil69420.github.io/renoir/).

Mix and match sensor hardware (IMU, GNSS, barometer, magnetometer, rangefinder, camera) from consumer to survey grade, fly in generated biomes with wind, motor vibration and temperature effects, and put a real flight stack in the loop: ArduPilot, PX4 or Betaflight run as software-in-the-loop against the simulator in lockstep.

```bash
./build/hummingbird --list-sensors
./build/hummingbird --headless --scenario scenarios/desert_rtk.scn
```

The source, build instructions and issue tracker are on [GitHub](https://github.com/Nil69420/mujoco_drones). The pages in this book describe what each part does, how it is tested, and where its limits are.
