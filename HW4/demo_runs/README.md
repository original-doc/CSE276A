# HW4 demo runs (selected)

Both runs on branch `HW4_zzyzhbs` at `df0da21` (r_margin 0.06, guard_margin 0.03; fast uses
the `fast:` overrides in `nav.yaml`: wp_spacing 0.8, undershoot_c 0.03, pass_pos 0.12, goal_pos 0.08).
Copies of `HW4/logs/<run>`; start pose in `config.yaml` is the default S (0.5, 2.5, -45 deg).

| run | planner | time | drives / spins / looks | replans | closest to box (centre) | final estimate vs G |
|---|---|---|---|---|---|---|
| `run_20261122_041857_fast` | fast | 31.7 s | 4 / 3 / 11 | 0 | 0.34 m | 7.5 cm |
| `run_20261122_042134_safe` | safe | 106.8 s | 15 / 18 / 44 | 0 | 0.65 m | 5.2 cm |

No `end.yaml` (measured end position) yet: add one per run for `HW4/report_run.py`.
