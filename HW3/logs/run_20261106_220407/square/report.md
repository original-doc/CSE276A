## Phase `square`

![map](map.png)

![sigma](sigma.png)

- vertices reached: 15/15; motion: 15.29 m driven, 1155° turned, 30 drives + 34 spins

### Final map vs truth

Placement in the arena: best rigid fit (no start.yaml) (0.541, 1.735, -82.3°)

| tag | estimate (arena) | truth | 2σ ellipse (cm) | error, measured start (cm) | error, rigid fit (cm) | ψ error (°) |
|---|---|---|---|---|---|---|
| 0 | (0.998, -0.085, +92.0°) | (1.000, 0.000, +90.0°) | 7.0 × 5.4 | — | 8.5 | +2.0 |
| 1 | (2.044, -0.060, +91.6°) | (2.000, 0.000, +90.0°) | 8.0 × 5.4 | — | 7.5 | +1.6 |
| 2 | (3.034, 0.936, -179.2°) | (3.000, 1.000, +180.0°) | 12.2 × 9.2 | — | 7.3 | +0.8 |
| 3 | (3.085, 1.994, +178.3°) | (3.000, 2.000, +180.0°) | 14.6 × 8.2 | — | 8.5 | -1.7 |
| 4 | (2.045, 3.132, -91.4°) | (2.000, 3.000, -90.0°) | 15.5 × 7.9 | — | 14.0 | -1.4 |
| 5 | (1.011, 3.186, -93.6°) | (1.000, 3.000, -90.0°) | 16.0 × 9.3 | — | 18.6 | -3.6 |
| 6 | (-0.119, 1.973, +3.3°) | (0.000, 2.000, +0.0°) | 12.7 × 6.8 | — | 12.2 | +3.3 |
| 7 | (-0.099, 0.924, +1.2°) | (0.000, 1.000, +0.0°) | 10.6 × 6.4 | — | 12.5 | +1.2 |

- **mean error, best rigid fit: 11.1 cm** (rms 11.7, max 18.6 cm, mean |ψ| 2.0°)
- mean error after rigid fit **with scale**: 5.0 cm; the map is **1.066×** the truth (a uniform scale error, e.g. tag size or arena size)

### Lap by lap (lap 1 builds the map; later laps re-observe it)

| lap | landmarks | mean error (cm) | after rigid fit (cm) | mean σ (cm) | looks | update | init | gate | skip_init | mean d² |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 7 | — | 11.7 | 10.2 | 33 | 40 | 7 | 0 | 17 | 0.69 |
| 2 | 8 | — | 11.4 | 7.6 | 24 | 41 | 1 | 0 | 4 | 0.83 |
| 3 | 8 | — | 11.1 | 7.1 | 22 | 36 | 0 | 0 | 0 | 0.72 |

### Landmark cases (new / out of view / re-appeared)

| tag | first added (t s, dist m, lap) | skip_init | updates | gated | re-appeared |
|---|---|---|---|---|---|
| 0 | 1.0, 1.92, lap 1 | 0 | 22 | 0 | 4 |
| 1 | 1.0, 2.31, lap 1 | 0 | 19 | 0 | 5 |
| 2 | 20.3, 0.68, lap 1 | 0 | 8 | 0 | 3 |
| 3 | 16.5, 1.89, lap 1 | 2 | 12 | 0 | 2 |
| 4 | 27.9, 2.27, lap 1 | 0 | 14 | 0 | 2 |
| 5 | 125.2, 1.72, lap 2 | 9 | 5 | 0 | 1 |
| 6 | 45.0, 2.31, lap 1 | 2 | 21 | 0 | 2 |
| 7 | 62.5, 1.92, lap 1 | 8 | 16 | 0 | 3 |
