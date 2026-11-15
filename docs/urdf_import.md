# urdf_import — URDF/SDF airframe importer

`urdf_import` converts a real-world URDF or Gazebo SDF airframe (with its
mesh assets) into a self-contained MJCF airframe under
`model/airframes/<name>/`, plus the generated `airframe_spec_t`
registration (`src/vehicle/airframes/<name>_spec.c`) so the vehicle can
be selected from a `.scn` scenario exactly like `quad_x`.

This is **not** a general-purpose converter for arbitrary internet
models. It is scoped to what the three sourced test vehicles and the
`quad_x` regression require, and it leans on MuJoCo 3.5.0's native URDF
parsing (`mj_parseXML`) rather than re-implementing a parser.

## Usage

```
urdf_import <input-dir> <output-name> [OPTIONS]
```

Run from the repository root (the tool writes `model/` and
`src/vehicle/airframes/` relative to the current directory).

- `<input-dir>` — directory containing exactly one `.urdf` or `.sdf`
  file plus its mesh assets.
- `<output-name>` — lowercase alphanumeric/underscore airframe name.

Options:

- `--model FILE` — explicit model file (bypasses the directory scan).
- `--config FILE` — actuator override config (see below).
- `--class NAME` — `airframe_class` tag for the registry entry
  (default `rotor_multicopter`).

What it does, in order:

1. Detects `<robot>` (URDF) vs `<sdf>` (SDF). SDF input is translated
   by a narrow translator (links/inertial/visual/collision, revolute/
   fixed/continuous joints, poses); anything outside the subset prints
   a named warning instead of silently vanishing.
2. Resolves every mesh reference (`package://`, relative, absolute)
   against the input directory, copies the meshes into
   `model/airframes/<name>/meshes/` and rewrites the references to
   plain relative paths. (Note: MuJoCo cannot load `.dae` meshes; the
   sourced fixtures ship as `.obj` — see `SOURCES.md`.)
3. Parses the rewritten model with `mj_parseXML` and injects
   actuators per joint (see below).
4. Synthesizes the URDF `<visual>` mesh geometry back into the model
   (MuJoCo's URDF import silently drops visual meshes).
5. Adds the Stage-1 sensor contract (imu/lidar/infrared sites plus the
   imu_accel/gyro/orientation/position/heading/linvel/mag sensor set)
   and a `base_link` reference body, then wraps the worldbody contents
   in a `base_link` body with a `root` freejoint (with the root link's
   authored inertial — MuJoCo's URDF import otherwise silently drops
   the root link mass).
6. Validates: per-body inertia checks, mesh-presence checks, and a 2 s
   at-rest settle (NaN/Inf and second-half energy drift). Any failure
   is a real bug in the pipeline — do not ship a failing import.
7. Writes the final self-contained MJCF and emits
   `src/vehicle/airframes/<name>_spec.c` plus the registry wiring to
   paste into `src/vehicle/airframe_registry.c` (and add the file to
   `SOURCES` in `CMakeLists.txt`).

## Actuator heuristic

Per joint, unless overridden by the config:

- Name contains `rotor` or `prop` → motor. Motors are converted to
  site-based lift (force along the joint axis at a
  `<joint>_thrust_site` site) with the yaw gear from the config.
- Finite joint range (`range[0] < range[1]`) → position servo, with
  inertia-aware default gains (kp = I_min·2500, kv = I_min·100;
  configured `kp`/`kv` are used verbatim).
- Otherwise the joint is reported undriven.

## Config file

INI-style, `#` comments:

```
[channels]
order = joint_a, joint_b, joint_a_spin   ; explicit channel order (required
                                          ; for index-based controllers like
                                          ; fixedwing_basic / pd_multicopter)

[joint rotor_0_joint]
type = motor
torque = -0.016      ; yaw gear (gear[5])
ctrlrange = 0 6

[joint elevon_left]
type = position
kp = 0.0025
kv = 0.0001

[spin rotor_6_joint]     ; adds a velocity (spin) actuator for the joint
type = velocity          ; and suppresses the default motor heuristic
kv = 0.0001
ctrlrange = -838 0
```

- `[channels] order = ...` lists actuator names in the order the
  controller expects (thrust channels first, then spins). Names not in
  the list are appended in creation order; unknown names warn.
- `[joint x]` and `[spin x]` may coexist; config entries override the
  heuristic; a `[spin x]` without a `[joint x]` makes the joint
  spin-only (no main actuator).
- A config joint name that does not exist in the model prints a
  warning; malformed configs fail loudly with line numbers.

## Known scope limits

- Not a general URDF/SDF parser: only the elements the sourced
  fixtures use, with loud warnings for everything else.
- Collision meshes are dropped (with a warning) — MuJoCo's native URDF
  import cannot load them.
- MuJoCo 3.5.0 merges the URDF root link into the world; the importer
  restores its inertial on the wrapper `base_link` body, but other
  root-link semantics (e.g. its frame) follow the import's behavior.
- Mesh assets do not transfer through `mjs_attach`: imported vehicles
  keep their physics (and sensors) in composed scenes, but their mesh
  visuals will not render there.
- The lidar ray filter on imported airframes covers only the
  `base_link` reference body (own-geom occlusion is not filtered).
- No automatic aerodynamic tuning: fixed-wing imports (e.g. the ASL
  Skywalker) need fluid surfaces added per vehicle to achieve
  controlled flight; the OMAV's thrust-vectoring controller
  (`omav_basic`) lifts but needs per-vehicle gain tuning. The
  `quad_x` URDF regression (scenarios/single_quadx.scn) is the
  reference proof that the pipeline produces drivable multicopters.
