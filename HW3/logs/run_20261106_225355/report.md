# HW3 run `run_20261106_225355`

## Phase `square`

![map](square/map.png)

![sigma](square/sigma.png)

- vertices reached: 13/15; motion: 14.75 m driven, 1134° turned, 26 drives + 32 spins

### Final map vs truth

Placement in the arena: best rigid fit (no start.yaml) (0.656, 1.809, -86.2°)

| tag | estimate (arena) | truth | 2σ ellipse (cm) | error, measured start (cm) | error, rigid fit (cm) | ψ error (°) |
|---|---|---|---|---|---|---|
| 0 | (0.969, -0.070, +90.2°) | (1.000, 0.000, +90.0°) | 7.0 × 5.5 | — | 7.7 | +0.2 |
| 1 | (2.015, -0.071, +90.0°) | (2.000, 0.000, +90.0°) | 8.0 × 5.3 | — | 7.3 | -0.0 |
| 2 | (3.081, 0.950, +178.7°) | (3.000, 1.000, +180.0°) | 12.8 × 7.4 | — | 9.5 | -1.3 |
| 3 | (3.109, 2.020, +178.0°) | (3.000, 2.000, +180.0°) | 15.0 × 7.1 | — | 11.1 | -2.0 |
| 4 | (2.025, 3.113, -87.5°) | (2.000, 3.000, -90.0°) | 17.0 × 7.6 | — | 11.5 | +2.5 |
| 5 | (0.964, 3.099, -88.7°) | (1.000, 3.000, -90.0°) | 16.2 × 7.7 | — | 10.6 | +1.3 |
| 6 | (-0.083, 2.006, +1.7°) | (0.000, 2.000, +0.0°) | 13.3 × 6.7 | — | 8.3 | +1.7 |
| 7 | (-0.079, 0.954, +1.1°) | (0.000, 1.000, +0.0°) | 10.8 × 5.8 | — | 9.2 | +1.1 |

- **mean error, best rigid fit: 9.4 cm** (rms 9.5, max 11.5 cm, mean |ψ| 1.3°)
- mean error after rigid fit **with scale**: 2.0 cm; the map is **1.059×** the truth (a uniform scale error, e.g. tag size or arena size)

### Lap by lap (lap 1 builds the map; later laps re-observe it)

| lap | landmarks | mean error (cm) | after rigid fit (cm) | mean σ (cm) | looks | update | init | gate | skip_init | mean d² |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 7 | — | 8.7 | 10.8 | 21 | 26 | 7 | 0 | 9 | 0.90 |
| 2 | 8 | — | 9.0 | 7.8 | 23 | 49 | 1 | 0 | 2 | 0.65 |
| 3 | 8 | — | 9.4 | 7.1 | 29 | 54 | 0 | 0 | 0 | 0.51 |

### Landmark cases (new / out of view / re-appeared)

| tag | first added (t s, dist m, lap) | skip_init | updates | gated | re-appeared |
|---|---|---|---|---|---|
| 0 | 1.0, 1.93, lap 1 | 0 | 21 | 0 | 4 |
| 1 | 1.0, 2.31, lap 1 | 0 | 23 | 0 | 6 |
| 2 | 16.6, 1.41, lap 1 | 0 | 7 | 0 | 4 |
| 3 | 16.6, 1.89, lap 1 | 2 | 15 | 0 | 4 |
| 4 | 25.3, 2.20, lap 1 | 0 | 13 | 0 | 2 |
| 5 | 88.8, 1.79, lap 2 | 7 | 9 | 0 | 3 |
| 6 | 37.6, 2.31, lap 1 | 0 | 19 | 0 | 2 |
| 7 | 43.8, 1.91, lap 1 | 2 | 22 | 0 | 4 |

## Phase `octagon`

![map](octagon/map.png)

![sigma](octagon/sigma.png)

- vertices reached: 23/24; motion: 12.29 m driven, 1087° turned, 25 drives + 37 spins

### Final map vs truth

Placement in the arena: best rigid fit (no start.yaml) (0.669, 1.766, -88.1°)

| tag | estimate (arena) | truth | 2σ ellipse (cm) | error, measured start (cm) | error, rigid fit (cm) | ψ error (°) |
|---|---|---|---|---|---|---|
| 0 | (0.971, -0.082, +89.4°) | (1.000, 0.000, +90.0°) | 6.7 × 5.3 | — | 8.7 | -0.6 |
| 1 | (2.010, -0.098, +89.2°) | (2.000, 0.000, +90.0°) | 7.7 × 5.1 | — | 9.8 | -0.8 |
| 2 | (3.090, 0.986, -179.3°) | (3.000, 1.000, +180.0°) | 12.0 × 6.3 | — | 9.1 | +0.7 |
| 3 | (3.108, 2.027, +178.6°) | (3.000, 2.000, +180.0°) | 14.0 × 6.1 | — | 11.1 | -1.4 |
| 4 | (2.024, 3.093, -89.0°) | (2.000, 3.000, -90.0°) | 14.8 × 7.0 | — | 9.6 | +1.0 |
| 5 | (0.972, 3.094, -89.9°) | (1.000, 3.000, -90.0°) | 14.4 × 7.2 | — | 9.8 | +0.1 |
| 6 | (-0.077, 2.012, -0.1°) | (0.000, 2.000, +0.0°) | 12.3 × 6.2 | — | 7.8 | -0.1 |
| 7 | (-0.097, 0.968, -0.6°) | (0.000, 1.000, +0.0°) | 10.1 × 5.7 | — | 10.2 | -0.6 |

- **mean error, best rigid fit: 9.5 cm** (rms 9.6, max 11.1 cm, mean |ψ| 0.7°)
- mean error after rigid fit **with scale**: 1.2 cm; the map is **1.060×** the truth (a uniform scale error, e.g. tag size or arena size)

### Lap by lap (lap 1 builds the map; later laps re-observe it)

| lap | landmarks | mean error (cm) | after rigid fit (cm) | mean σ (cm) | looks | update | init | gate | skip_init | mean d² |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 8 | — | 10.7 | 9.6 | 32 | 46 | 8 | 0 | 0 | 0.57 |
| 2 | 8 | — | 9.6 | 6.9 | 29 | 54 | 0 | 0 | 0 | 0.61 |
| 3 | 8 | — | 9.5 | 6.5 | 25 | 33 | 0 | 0 | 0 | 0.69 |

### Landmark cases (new / out of view / re-appeared)

| tag | first added (t s, dist m, lap) | skip_init | updates | gated | re-appeared |
|---|---|---|---|---|---|
| 0 | 1.0, 1.89, lap 1 | 0 | 18 | 0 | 6 |
| 1 | 1.0, 2.30, lap 1 | 0 | 22 | 0 | 3 |
| 2 | 25.2, 1.33, lap 1 | 0 | 5 | 0 | 3 |
| 3 | 19.7, 2.34, lap 1 | 0 | 17 | 0 | 2 |
| 4 | 27.6, 2.35, lap 1 | 0 | 21 | 0 | 4 |
| 5 | 35.7, 2.31, lap 1 | 0 | 20 | 0 | 4 |
| 6 | 53.1, 2.29, lap 1 | 0 | 16 | 0 | 4 |
| 7 | 61.1, 2.20, lap 1 | 0 | 14 | 0 | 2 |

## Square vs octagon (final maps)

| phase | laps | landmarks | mean error (cm) | after rigid fit (cm) | with scale (cm) | map / truth | lap 1 error (cm) | mean σ lap 1 → last (cm) | drive (m) | turns |
|---|---|---|---|---|---|---|---|---|---|---|
| square | 3 | 8 | — | 9.4 | 2.0 | 1.059 | — | 10.8 → 7.1 | 14.7 | 32 |
| octagon | 3 | 8 | — | 9.5 | 1.2 | 1.060 | — | 9.6 → 6.5 | 12.3 | 37 |
