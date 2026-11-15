# Estimation

`estimator = kf_ins` is a loosely coupled INS:

- **Attitude**: a Mahony filter. The accelerometer reference is corrected with the acceleration derived from
  the velocity estimate (otherwise a coordinated turn tilts the apparent vertical), and the magnetometer only
  corrects yaw, so motor interference cannot tilt the attitude. Attitude initialises from the accelerometer when
  the vehicle is at rest; after 0.25 s without a usable reading (a mid-air start) it assumes level. Yaw starts
  from the magnetometer assuming the horizontal field points to +Y (north) in the simulator frame, so the
  magnetic declination (a few degrees) is not modelled.
- **Position and velocity**: a three-state filter per axis (position, velocity, accelerometer bias) driven by
  the rotated accelerometer, with a barometer-bias state on the vertical axis. GNSS position and velocity are
  fused with their **measurement** timestamp against a history of past states, so latency is compensated.
- The estimate is anchored to the simulator's geodetic origin, so airborne spawns work.

Accuracy is limited by the sensors: correlated GNSS error cannot be filtered out (the filter is no worse
than raw GNSS and is far better with uncorrelated noise), and a consumer receiver in a city canyon has several
metres of error. `tests/test_kf_ins.c` states the measured numbers.

`complementary_filter` is kept for compatibility; it copies GNSS position and velocity directly and is only
suitable with `gnss = ideal`.
