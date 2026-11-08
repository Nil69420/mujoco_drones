# Sensors

Every sensor is a pure model in `src/sensors/models/` driven by a named hardware profile, so a vehicle's
sensor suite can be mixed and matched from the command line or a scenario file. `hummingbird --list-sensors`
prints what is installed.

| Kind | Option | Profiles | Rate option (default) |
|------|--------|----------|----------------------|
| IMU | `--imu` | `mpu6050` (consumer), `bmi088`, `icm42688p` (industrial), `adis16470` (tactical-lite), `stim300` (tactical), `kvh1750` (navigation-lite) | `--imu-rate` (1000 Hz) |
| GNSS | `--gnss` | `ideal`, `neo_m8n` (consumer), `zed_f9p` (survey, RTK capable) | `--gnss-rate` (10 Hz) |
| Barometer | `--baro` | `ideal`, `ms5611`, `bmp388`, `dps310` | `--baro-rate` (50 Hz) |
| Magnetometer | `--mag` | `ideal`, `hmc5883l`, `rm3100` | `--mag-rate` (min(100 Hz, profile maximum)) |
| Rangefinder | `--rangefinder` | `ideal`, `tfmini_s`, `tf02_pro`, `lidar_lite_v3`, `sf11_c` | `--rangefinder-rate` (100 Hz) |
| Camera | `--cam-profile` | `ideal`, `ar0234`, `ov9281` | `--cam-fps` (8 Hz) |

Without a profile the legacy fixed Gaussian noise is used. An unknown profile, or a rate that the profile or
the simulation step cannot deliver (for example a 2000 Hz IMU on the 1 kHz step), is a hard error rather than a
silent fallback.

## What is modelled

- **IMU**: white noise (density), bias instability as a first-order Gauss-Markov process, bias random walk,
  turn-on bias, scale factor and axis misalignment errors, temperature coefficient, g-sensitivity, bandwidth,
  saturation. Profiles are checked against their Allan deviation in `tests/test_imu_model.c`.
- **GNSS**: correlated plus white position error scaled by multipath and sky visibility, velocity noise, cold
  start (time to first fix), measurement latency, fix states and RTK convergence (`--gnss-rtk`, only available
  with enough sky and little multipath). Reported satellites and HDOP follow visibility.
- **Barometer**: noise, slow drift, power-on offset, temperature coefficient, propwash from motor thrust,
  resolution.
- **Magnetometer**: Earth field rotated into the body frame, hard and soft iron, motor interference that
  scales with battery current, temperature coefficient, range and resolution.
- **Rangefinder**: reflectivity-dependent maximum range (so snow and asphalt behave differently), near and far
  noise, resolution and dropouts near the limit.
- **Camera**: see [camera.md](camera.md).

## Conditions that act on the sensors

`--weather` (or `weather =` in a scenario) selects ambient temperature, pressure and wind. Rotor vibration
(`--vibration 1`) shakes the IMU through a damped mount model; sensor self-heating follows a first-order
thermal model and feeds every temperature coefficient and the camera.

## Provenance of the numbers

Each profile carries `source`, `accuracy_provenance` and `other_provenance` fields. A value is marked
`datasheet` only when it was read directly from a manufacturer document, `secondary` when it comes from a
reseller listing or was derived from published figures (for example camera full-well capacity and read noise
derived from the quoted dynamic range and peak SNR), and `assumed` when no source was found (dropout rates,
10 % reflectivity ranges for some rangefinders, camera dark current for the AR0234). Treat `assumed` values
as placeholders until a datasheet is checked.
