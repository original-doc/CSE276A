# `dev-yuanshengqi` branch-only archive

This branch preserves individual field experiments and robot-specific operating
notes. Do **not** merge the whole branch into `main`. Git has no reliable
per-file "never merge" flag; integrate reviewed commits or paths explicitly.

As of 2026-10-14, keep these paths on this branch only:

- `calibration/` — earlier IMU/magnetometer experiments, raw samples, and
  one-off calibration scripts. The shared calibration implementation is in
  `main` under `robot_control/robot_control/` and `HW1/`.
- `robot_control/config/floors/carpet2.yaml` and
  `robot_control/config/diag/imu_diag_carpet2_20261014_180848.csv` — personal,
  partly reconstructed carpet2 observations; not a team-approved floor profile.
- `HANDOFF.md` — dated personal handoff and machine-specific operating notes.
- `ops/` — hotspot recovery service tied to one robot and the saved `Kissshot`
  Wi-Fi profile. It needs team review and configuration before reuse elsewhere.
- This `BRANCH_ONLY.md` inventory itself.

The reviewed shared addition is `tools/` (the standalone SSH keyboard teleop,
tests, and usage guide), originally committed as `aa2e2d7`. Integrate that
commit selectively; do not use `git merge dev-yuanshengqi` for this scope.

This file is a documented merge decision, **not** a Git enforcement mechanism.
Revisit the decision if the team later wants any archived experiment or ops tool.
