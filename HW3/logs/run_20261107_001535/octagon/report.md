## Phase `octagon`

![map](map.png)

![sigma](sigma.png)

- vertices reached: 24/24; motion: 13.88 m driven, 1282° turned, 41 drives + 34 spins

### Final map vs truth

Placement in the arena: measured start (0.750, 1.810, -90.0°)

| tag | estimate (arena) | truth | 2σ ellipse (cm) | error, measured start (cm) | error, rigid fit (cm) | ψ error (°) |
|---|---|---|---|---|---|---|
| 0 | (1.127, 0.073, +91.4°) | (1.000, 0.000, +90.0°) | 6.1 × 4.8 | 14.7 | 1.0 | +1.4 |
| 1 | (2.110, 0.111, +92.5°) | (2.000, 0.000, +90.0°) | 7.0 × 4.7 | 15.6 | 2.1 | +2.5 |
| 2 | (3.081, 1.138, -176.4°) | (3.000, 1.000, +180.0°) | 10.4 × 5.3 | 16.0 | 0.8 | +3.6 |
| 3 | (3.061, 2.134, -179.4°) | (3.000, 2.000, +180.0°) | 12.1 × 5.6 | 14.7 | 1.0 | +0.6 |
| 4 | (2.007, 3.110, -86.5°) | (2.000, 3.000, -90.0°) | 13.4 × 6.5 | 11.0 | 1.2 | +3.5 |
| 5 | (1.031, 3.076, -88.5°) | (1.000, 3.000, -90.0°) | 13.3 × 6.8 | 8.3 | 2.3 | +1.5 |
| 6 | (0.057, 1.995, +2.9°) | (0.000, 2.000, +0.0°) | 11.2 × 5.5 | 5.7 | 2.9 | +2.9 |
| 7 | (0.094, 1.010, +2.1°) | (0.000, 1.000, +0.0°) | 9.6 × 4.9 | 9.5 | 1.4 | +2.1 |

- **mean error, measured start: 11.9 cm** (rms 12.5, max 16.0 cm, mean |ψ| 2.3°)
- **mean error, best rigid fit: 1.6 cm** (rms 1.8, max 2.9 cm, mean |ψ| 0.8°)
- mean error after rigid fit **with scale**: 1.6 cm; the map is **0.999×** the truth (a uniform scale error, e.g. tag size or arena size)

### Lap by lap (lap 1 builds the map; later laps re-observe it)

| lap | landmarks | mean error (cm) | after rigid fit (cm) | mean σ (cm) | looks | update | init | gate | skip_init | mean d² |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 8 | 13.0 | 3.9 | 7.2 | 36 | 57 | 8 | 0 | 1 | 0.37 |
| 2 | 8 | 12.1 | 1.8 | 6.1 | 30 | 61 | 0 | 0 | 0 | 0.81 |
| 3 | 8 | 11.9 | 1.6 | 5.9 | 33 | 37 | 0 | 0 | 0 | 0.66 |

### Landmark cases (new / out of view / re-appeared)

| tag | first added (t s, dist m, lap) | skip_init | updates | gated | re-appeared |
|---|---|---|---|---|---|
| 0 | 1.0, 1.79, lap 1 | 0 | 19 | 0 | 4 |
| 1 | 1.0, 2.16, lap 1 | 0 | 19 | 0 | 5 |
| 2 | 19.8, 1.91, lap 1 | 0 | 14 | 0 | 4 |
| 3 | 17.4, 2.39, lap 1 | 0 | 22 | 0 | 6 |
| 4 | 40.2, 2.00, lap 1 | 0 | 21 | 0 | 7 |
| 5 | 45.0, 2.26, lap 1 | 0 | 17 | 0 | 4 |
| 6 | 63.4, 1.85, lap 1 | 0 | 22 | 0 | 8 |
| 7 | 63.4, 2.18, lap 1 | 1 | 21 | 0 | 6 |
