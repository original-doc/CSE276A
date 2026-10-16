# `dev-yuanshengqi` integration inventory

Updated 2026-10-15. This branch now includes the `open_loop` framework through
merge commit from `a3f578e`. Continue development here; the paths below are the
work that was unique to `dev-yuanshengqi` before that merge. These labels are
documentation, not Git merge rules.

| Label | Paths | Value and current role |
| --- | --- | --- |
| **ACTIVE AUXILIARY** | `tools/` | Standalone SSH keyboard/timed-motion and human-referenced turn/straight tools, usage guide, and four tests. Useful for controlled manual experiments; they are not the `ol_calibrate` or `velocity_control` implementation. Run only one `motor_commands` publisher at a time. |
| **ROBOT-SPECIFIC** | `ops/` | NetworkManager hotspot recovery timer and script. Keep as a troubleshooting/deployment reference; it is configured for this robot's `wlan0` and `Kissshot` profile, not a portable project default. |
| **EXPERIMENT ARCHIVE** | `calibration/`, `robot_control/config/floors/carpet2.yaml`, `robot_control/config/diag/imu_diag_carpet2_20261014_180848.csv` | Raw observations, analyses, and older IMU/magnetometer-based experiments, including marble and carpet2. Preserve their provenance; do not feed their parameters into the selected open-loop wood-floor controller without new validation. |
| **HISTORICAL HANDOFF** | `HANDOFF.md` | Snapshot written on 2026-10-13. It predates the `open_loop` framework and current wood-floor trials; it is not the current runbook. |

The selected control path is the merged `robot_control/robot_control/ol_*` code,
`velocity_control.py`, and the waypoint/configuration files from `open_loop`.
`robot_control/config/open_loop/wood.yaml` is only a template. The measured
`study_wood.yaml` and `study_wood2.yaml` were saved separately in
`codex/study-wood-calibration`; that cloud snapshot lacks later robot-side
`verify` records and demo logs. Recover the latest robot copy before treating
either profile as the active floor calibration in this branch.

Do not merge this entire branch into `main` just to transfer the auxiliary or
historical files. Review the intended paths first. Conversely, merging
`open_loop` into this branch is intentional and does not make those archived
files part of the open-loop runtime. Preserve this inventory when deciding
which files should eventually be shared with the team.
