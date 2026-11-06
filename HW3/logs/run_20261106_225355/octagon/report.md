## Phase `octagon`

![map](map.png)

![sigma](sigma.png)

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
