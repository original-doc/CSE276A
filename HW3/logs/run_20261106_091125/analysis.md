# Run `run_20261106_091125`: analysis

Full run: square × 3 laps → filter reset → octagon × 3 laps, no halt.
Generated with `HW3/report_run.py` (`report.md`, `square/`, `octagon/` figures and tables).

## Caveats (fix before quoting final numbers)

- `HW3/truth_map.yaml` still holds the **nominal** tag poses (1 m / 2 m on each side of a 3 m arena),
  not tape measurements.
- `square/start.yaml` is **approximate** (placed "about (0.75, 1.81)", facing −y), not measured.
- `octagon/start.yaml` is missing (the pose was not measured during the pause), so the octagon
  map is placed by the best rigid fit only.
- After updating these files, re-run `/opt/anaconda3/bin/python HW3/report_run.py HW3/logs/run_20261106_091125`.

## Final maps vs truth

| phase | landmarks | mean error, measured start | after rigid fit | after rigid fit + scale | map / truth scale |
|---|---|---|---|---|---|
| square | 8 / 8 | 14.3 cm | 10.7 cm | **3.5 cm** | 1.065 |
| octagon | 8 / 8 | — (no start.yaml) | 9.2 cm | **1.6 cm** | 1.057 |

- Both maps are **uniformly ~6 % larger** than the truth map; once a single scale factor is removed
  the shape error is 1.6–3.5 cm. A uniform scale points to one of:
  1. the real tag positions (arena size) differ from the nominal truth map, or
  2. the printed tag size differs from `size: 0.166` in `config/hw3/apriltag.yaml`
     (PnP distance scales with the tag size: a 0.176 m tag configured as 0.166 m makes every
     distance ~6 % too long). The HW2 log replay also found tag 0 at 1.61 m vs 1.50 m in the
     hand map (+7 %).
  → Measure the black square edge of a printed tag and the tag positions with a tape.
- Heading errors of the tags: mean |ψ| 2.6° (square), 1.0° (octagon).

## Lap by lap

| phase | lap | landmarks | error, measured start | after rigid fit | after rigid fit + scale | mean σ |
|---|---|---|---|---|---|---|
| square | 1 | 5 | 17.1 cm | 15.4 cm | 8.7 cm | 36.5 cm |
| square | 2 | 8 | 13.0 cm | 11.6 cm | 4.5 cm | 22.2 cm |
| square | 3 | 8 | 14.3 cm | 10.7 cm | 3.5 cm | 19.8 cm |
| octagon | 1 | 8 | — | 9.1 cm | 1.5 cm | 7.6 cm |
| octagon | 2 | 8 | — | 9.2 cm | 1.5 cm | 6.4 cm |
| octagon | 3 | 8 | — | 9.2 cm | 1.6 cm | 6.1 cm |

- Square: laps 2–3 improve the map (shape error 8.7 → 3.5 cm, σ 36.5 → 19.8 cm). Lap 1 mapped only
  5 tags; tags 3, 5, 7 were first added in lap 2 (skipped earlier as too far / too oblique).
- Octagon: all 8 tags were mapped in lap 1 and the map barely changes afterwards (σ 7.6 → 6.1 cm).
- No observation was rejected by the gate in either phase (`gate` = 0 in every lap).

## Why the square map is less certain than the octagon map

- The first look of the square phase (at the start) got only 1 frame per tag (camera frames
  dropped), so no tag was added at the start. The robot drove 0.75 m on prediction only and tag 0
  was added after that drive, carrying the drive uncertainty (~16 cm with `k_d` 0.2). The whole
  square map is anchored to that first tag, so its σ never drops below ~18 cm.
- The octagon phase added tags 0 and 1 from its very first look, so its map is anchored with the
  observation noise only (σ ≈ 6–7 cm).

## Other observations

- Square: 4 of 15 vertices and octagon: 1 of 24 vertices are marked "not reached" with
  `no calibrated level` for clockwise turns of 26–32°: with the cw `time_scale` 0.77 of the
  calibration file the coarse level needs T < 0.2 s there. The robot then went on to the next vertex.
- Several looks still got only 1–2 frames per tag while the robot was standing still (image drops
  between the camera and the AprilTag node).
