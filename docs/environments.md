# Environments (biomes)

A scenario's `environment` key selects a biome. Each biome is data (`src/environments/biome_profiles.c`): a
terrain recipe, ground colour and friction, scatter rules and climate defaults. The generator
(`src/environments/biome*.c`, `terrain.c`) is seeded, so the same `environment_seed` always produces the same
world. The spawn area is kept flat and clear.

| Biome | Description |
|-------|-------------|
| `desert` | Dunes, scattered rock, cacti and scrub |
| `forest` | Temperate mixed forest on rolling hills |
| `jungle` | Dense tropical canopy with palms and broadleaf trees |
| `savanna` | Open grassland with sparse trees |
| `agricultural` | Crop fields, farm buildings, fences and power poles |
| `rural` | Village with scattered houses, trees, fields and a few roads |
| `urban` | Mid-rise city blocks on a street grid |
| `city` | Dense downtown with high-rise towers and urban canyons |
| `mountains` | Alpine ridges and valleys with conifers and boulders |
| `snow` | Snow-covered alpine terrain with conifers and rocks |
| `tundra` | Flat treeless permafrost plain with low shrubs and rocks |
| `ocean` | Open sea with swell, ships and rock outcrops |
| `coast` | Beach and shoreline sloping into the sea |
| `swamp` | Low wetland with standing water, reeds and trees |
| `canyon` | Arid mesa and canyon terrain with boulders |

Each biome also sets climate defaults that vehicles inherit unless the scenario overrides `weather`:
a weather preset, GNSS sky visibility and multipath scale (dense cities have both poor visibility and heavy
multipath, so GNSS error grows and RTK cannot engage), ground reflectivity for the rangefinder, visibility
range (fog) and sky colours.

Terrain is a heightfield; trees, rocks, buildings, ships and crop rows are simple primitives, so the worlds
are suited to flight dynamics, sensor and navigation testing rather than photorealism. Foliage does not
collide; trunks, rocks and buildings do.

Add a biome by adding an entry to `BIOME_PROFILES`; the registry and `tests/test_biomes.c` pick it up.
