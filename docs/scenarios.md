# Scenario files

```
hummingbird --headless --scenario scenarios/desert_rtk.scn
```

```ini
[scenario]
environment = desert          # a biome name (see environments.md)
environment_seed = 5
weather = hot_desert          # optional; defaults to the biome's climate
duration = 70.0

[vehicle rtk0]
airframe = quad_x
controller = pd_multicopter
estimator = kf_ins            # see estimation.md
localization = gnss
spawn = 0.0 0.0 0.3
target = 0.0 0.0 10.0         # optional; defaults to the spawn point
yaw = 0.0
imu = adis16470               # sensor profiles, see sensors.md
gnss = zed_f9p
gnss_rtk = true               # RTK corrections (needs open sky)
gnss_start = warm             # or cold: no fix until the receiver's time to first fix
baro = dps310
mag = rm3100
rangefinder = tf02_pro
vibration = 1.0               # motor vibration scale
drag_area = 0.06              # Cd*A in m^2 for wind drag on rotor airframes
```

Wind (mean flow with height shear plus Dryden-style turbulence) acts on rotor airframes through a quadratic
drag force whenever the weather preset has wind. A vehicle keeps its motors idle until its estimator is
valid, as a flight controller would.

Included scenarios: `single_quad`, `two_vehicle`, `single_quadx`, `single_asl`, `single_omav` (imported
airframes), `city_survey` (urban canyon with RTK requested, which cannot engage there), `budget_forest`
(consumer sensors) and `desert_rtk` (takeoff, then centimetre-level hold once RTK converges).
