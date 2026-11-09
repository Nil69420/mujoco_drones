# Airframes

`hummingbird --list-airframes` is not available yet; the registry in `src/vehicle/airframe_registry.c` is the list. Select an airframe per vehicle in a scenario with `airframe = <name>`.

| Name | Class | Rotors / surfaces | Notes |
|---|---|---|---|
| `quad_x` | rotor_multicopter | 4 rotors on the axes (plus layout) | Reference quad, 0.68 kg |
| `hexa` | rotor_multicopter | 6 rotors every 60 degrees | Generated, same mass and arm length as `quad_x` |
| `octo` | rotor_multicopter | 8 rotors every 45 degrees | Generated, same mass and arm length as `quad_x` |
| `quadx` | rotor_multicopter | Imported from a URDF | Importer golden-test fixture |
| `omav` | rotor_multicopter | 6 tilting rotors (Voliro OMAV) | Imported |
| `aslfixedwing` | fixed_wing_conventional | Ailerons, elevator, rudder, propeller | Stanford ASL Skywalker (MIT) |
| `fixedwing_basic` | fixed_wing_conventional | Basic flying wing | Minimal model |

## Generated multirotors

`tools/gen_multirotor.py` writes `model/airframes/hexa.xml` and `octo.xml`: rotors evenly spaced starting on +X, alternating spin, the same body mesh, 0.68 kg take-off mass and 0.17 m arm as `quad_x`. The existing `pd_multicopter` controller handles up to eight rotors, so the new airframes need no controller code. `tests/multirotor_live.sh` checks that the committed files match the generator and that `quad_x`, `hexa` and `octo` all reach and hold a 3 m target (final position within 3 m horizontally and 1 m vertically; the reference quad's own settling error is about 1.5 m at 14 s).

To add a different count or mass, extend the table in the script's `main` and add a matching `airframe_spec_t` in the registry (channel names `thrust_i` then `spin_i`). Odd rotor counts are not supported by the controller's alternating-spin mixer.

## Not yet available

Tricopter, coaxial and Y6 frames, helicopter, tailsitter and tiltrotor VTOL, flying wing with an aerodynamic model, a loitering fixed-wing airframe and foldable (morphing) multirotors are planned in the roadmap (workstream B). Each needs either a new mixer (non-uniform geometry) or an aerodynamic model; none of them is claimed here.
