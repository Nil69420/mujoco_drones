# Airframe effects

Four terms that no sensor datasheet gives you, because they belong to one airframe and have to be measured on it:

| Term | Meaning |
|------|---------|
| accelerometer thrust deficit | Vertical specific force the accelerometer under-reads in proportion to thrust (m/s² at full thrust) |
| barometer lag | First-order lag between the true pressure and the reading (s) |
| camera frame-drop probability | Probability that a rendered frame is never published; the sequence number still advances, so consumers see a gap |
| camera time shift | Offset added to the image timestamp relative to the IMU (s, may be negative) |

Vibration is separate: it is already modelled from rotor speed (`--vibration`, `vibration =`).

## No defaults

A simulator that fills these in with plausible numbers looks configured and cannot show the failures they cause. There is therefore no default: either all four are given, or none. A partial set is refused with a message naming the missing options, every value is range-checked (a non-finite or out-of-range value is rejected), and an effect that needs a model you did not select is refused as well: the deficit needs an IMU profile, the lag a barometer profile, and the camera terms a camera profile.

Command line (hummingbird):

```bash
./build/hummingbird --headless --imu icm42688p --baro bmp388 \
    --fx-accel-deficit 0.4 --fx-baro-lag 0.25 --fx-cam-drop 0.02 --fx-cam-shift 0.012
```

Scenario file (per vehicle):

```ini
imu = icm42688p
baro = bmp388
fx_accel_deficit = 0.4
fx_baro_lag = 0.25
fx_cam_drop = 0.02
fx_cam_shift = 0.012
```

Values given this way are recorded with the provenance `assumed`.

## Where each effect is applied

- The deficit is subtracted from the vertical accelerometer axis, scaled by the commanded thrust fraction. It works with vibration off.
- The lag is applied to the true pressure before the barometer model adds its own noise and offset.
- A dropped camera frame is not published; the timestamp shift is applied to published frames. Both need the camera model (`--cam-profile`).

## Effect on a real autopilot

With the ArduPilot JSON bridge the modelled IMU is what ArduPilot flies on, so the deficit reaches the flight controller. Measured on stock ArduCopter 4.6.1, hovering at 5 m (one run each; the 1.0 case reproduced exactly on a repeat):

| deficit (m/s²) | RAW_IMU zacc (mG) | takeoff altitude (m) | hold altitude (m) | hold std (m) |
|---|---|---|---|---|
| 0 | −993.4 | 4.99 | 4.99 | 0.002 |
| 0.4 | −981.6 | 5.22 | 5.12 | 0.046 |
| 1.0 | −963.7 | 5.55 | 5.32 | 0.114 |
| 2.0 | −933.5 | 5.85 | 5.60 | 0.169 |

The shift in the IMU reading is the deficit scaled by the hover thrust fraction (about 0.29). The altitudes are what ArduPilot's EKF believes. `tests/ardupilot_effects_e2e.sh` flies the 0 and 1.0 cases and checks that the IMU shifts by 15 to 45 mG and the hold altitude by 0.1 to 1.0 m.

## Limits

- The barometer lag is applied to the barometer message, but ArduPilot SITL derives its barometer from the position the bridge reports, so the lag is not expected to reach the flight controller over the JSON backend. This has not been tested.
- The camera effects are covered by unit tests and the sensor manager, not by a live camera run.
- The values in the examples are placeholders, not measurements of any aircraft.
