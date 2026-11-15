# Third-party assets

Vehicle models under `model/airframes/` that were imported from other projects keep their original licenses. License texts are in `LICENSES/`.

| Directory | Source | License | Notes |
|---|---|---|---|
| `model/airframes/aslfixedwing/` | https://github.com/StanfordASL/asl_fixedwing (Skywalker X8 SDF and meshes) | MIT, Copyright (c) 2020 Joe Lorenzetti (`LICENSES/MIT-asl_fixedwing.txt`) | Converted to MJCF by the project's import tool. |
| `model/airframes/omav/` | Voliro OMAV description from https://github.com/ethz-asl/rotors_simulator (`rotors_description`) | Apache-2.0, Copyright 2015 Fadri Furrer, Michael Burri, Mina Kamel, Janosch Nikolic, Markus Achtelik, ASL, ETH Zurich (`LICENSES/Apache-2.0.txt`) | `omav_source.urdf` is the xacro-expanded upstream file and keeps its license header. Converted to MJCF by the import tool. |

Not included: the PX4 `standard_vtol` model. The upstream repository has no license file, so its meshes are not redistributed here.
