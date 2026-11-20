# HW4: path planning demo — how to run

Branch `HW4`.
Two runs, S → G around one box: `planner:=safe` (GVD + Dijkstra, max clearance) and
`planner:=fast` (visibility graph + Dijkstra, shortest). Each run: plan once → drive the plan
with stop-look-correct (known map, 12 tags) → replan if a drive would pass too close.

| File | What |
|---|---|
| `robot_control/config/hw4/world.yaml` | arena, box, robot size, margins, **start**, goal, 12 tag poses |
| `robot_control/config/hw4/nav.yaml` | levels, look, control, EKF noise, `plan:` and `exec:` parameters |
| `robot_control/config/hw4/apriltag.yaml` | apriltag node (ids 0–11, size 0.156) |
| `robot_control/launch/hw4_launch.py` | perception + `motor_control` + `hw4_nav` |
| `HW4/logs/run_<time>_<planner>/` | logs per run; you add `end.yaml` |
| `HW4/plot_plan.py` | planner figures: representation, path, expansion order (Mac) |
| `HW4/report_run.py` | run figures + tables, safe vs fast (Mac) |

## 0. Deploy

Robot:
```bash
cd ~/ros2_ws/src/CSE276A && git fetch && git checkout HW4 && git pull
cd ~/ros2_ws && colcon build --packages-select robot_control
source ~/ros2_ws/install/setup.bash        # in every new terminal
```

Rebuild after every pull, and after editing `apriltag.yaml` or launch files.
`world.yaml`, `nav.yaml`, calibration and logs are read from / written to the source folder
(no rebuild).

Before every launch:
```bash
pgrep -af robot_vision_camera_node         # if a PID shows: kill -9 <pid>
```

## 1. Field setup

Arena 3 m × 3 m, origin = bottom-left corner, x along the bottom edge, y along the left edge.
Tag centres at camera height (~12.5 cm), faces pointing away from the wall / box face.

Wall tags (same as HW3):

| tag | (x, y) m | faces |
|---|---|---|
| 0 | (1, 0) | +y |
| 1 | (2, 0) | +y |
| 2 | (3, 1) | −x |
| 3 | (3, 2) | −x |
| 4 | (2, 3) | −y |
| 5 | (1, 3) | −y |
| 6 | (0, 2) | +x |
| 7 | (0, 1) | +x |

Box: centre (1.5, 1.5), 0.35 m × 0.35 m, one tag in the middle of each face:

| tag | face | (x, y) m | faces |
|---|---|---|---|
| 8 | bottom | (1.5, 1.325) | −y |
| 9 | right | (1.675, 1.5) | +x |
| 10 | top | (1.5, 1.675) | +y |
| 11 | left | (1.325, 1.5) | −x |

Start S: robot centre (base_link) at (0.5, 2.5), facing −45° (towards G). Goal G: (2.5, 0.5).
Mark S with tape (centre + heading line).

Measure everything from the same corner and write it into `world.yaml`:
- `obstacles:` the real box (`cx, cy, w, h`)
- `landmarks:` every tag; box tags must sit on the box faces (≤ 1 cm) and face out (≤ 10°),
  or loading stops with `tag <id> is not on a face of obstacle 0`
- `robot: extra:` anything that sticks out past 194 × 168 mm (camera mount: 0)
- `start:` the measured S (`x, y, theta_deg`) — **before every run**

Check the file on the Mac (prints the radii and both plans):
```bash
cd robot_control
/opt/anaconda3/bin/python -c "
from robot_control.world import load_world
from robot_control.plan_safe import plan_safe
from robot_control.plan_fast import plan_fast
w = load_world('config/hw4/world.yaml')
print(f'r_robot {w.r_robot:.3f}  r_inflate {w.r_inflate:.3f}  r_guard {w.r_guard:.3f}')
for p in (plan_safe(w), plan_fast(w)):
    print(p.kind, f'{p.length:.3f} m, min clearance {p.min_clearance:.3f} m,', p.points)"
cd ..
```
Expected with the defaults: safe ≈ 3.72 m, fast ≈ 3.03 m (one corner at (1.117, 1.117)).

## 2. Planner figures (Mac, no robot)

```bash
/opt/anaconda3/bin/python HW4/plot_plan.py                 # [--tie-side left]
```
Outputs in `HW4/figures/`: `plan_safe.png` (distance field + GVD + path), `plan_fast.png`
(inflated box + visibility graph + path), `plan_compare.png`, `expand_safe.png`,
`expand_fast.png` (Dijkstra expansion order). Re-run after changing `world.yaml`.

## 3. Perception check (no motion)

T1:
```bash
ros2 launch robot_control hw2_perception_launch.py \
  apriltag_config:=$HOME/ros2_ws/install/robot_control/share/robot_control/config/hw4/apriltag.yaml
```

T2:
```bash
ros2 topic hz /detections
ros2 run tf2_ros tf2_echo base_link tag_10
```
From S facing −45° expect tags 0, 10, 11.

Stop T1 before step 4.

## 4. Dry run (never moves)

Robot on S.
```bash
ros2 launch robot_control hw4_launch.py planner:=safe dry_run:=true
ros2 launch robot_control hw4_launch.py planner:=fast dry_run:=true
```
Expect: the plan table (waypoints, length, min clearance), then 3 looks with the estimate close
to `start` (a few cm, a few deg) and the first planned step.

Foxglove: `ws://<robot-ip>:8765`, 3D panel, fixed frame `world`;
topics `/hw4/pose`, `/hw4/plan` (arena, box, inflated box, tags, path, waypoints);
image `/camera/image_raw/compressed`.

## 5. Short test

One run of each planner without video. Watch for `REPLAN` lines and how close the robot
gets to the box.
```bash
ros2 launch robot_control hw4_launch.py planner:=safe
ros2 launch robot_control hw4_launch.py planner:=fast
```

## 6. Demo runs (video)

Charge the battery first; run safe and fast one after the other.

For each planner:
1. Robot on S. Measure it; write `start:` in `world.yaml`.
2. Start the video, then:
   ```bash
   ros2 launch robot_control hw4_launch.py planner:=safe      # then planner:=fast
   ```
3. It plans, prints the plan, counts down 3 s, and drives to G. At G it turns to the nearest
   tag (tag 1 or 2) for the last look. Everything shuts down at the end. Ctrl-C stops the motors.
4. Robot stopped: tape-measure its centre; write `end.yaml` in the run's log dir (step 8).

Launch arguments (defaults):

| argument | default |
|---|---|
| `planner` | — (`safe` or `fast`, required) |
| `dry_run` | `false` |
| `floor` | `study_wood2_hw2_ext` |
| `tag_z_sign` | `1` |
| `wp_pause_s` | `1.0` |

Separate terminals instead (restart only the runner):
```bash
# T1
ros2 launch robot_control hw2_perception_launch.py \
  apriltag_config:=$HOME/ros2_ws/install/robot_control/share/robot_control/config/hw4/apriltag.yaml
# T2
ros2 run robot_control motor_control
# T3
ros2 run robot_control hw4_nav --ros-args -p planner:=safe
```

What the output means:

| line | meaning |
|---|---|
| `=== safe plan (r_inflate = 0.208 m): ...` | the plan; `r_guard` instead = start was too close for `r_inflate` |
| `>>> wp k (...) reached: estimate ...` | one waypoint done (every 0.5 m along the path) |
| `>>> REPLAN n: drive ... would pass too close` | a drive (counted 25 % long) would pass within `r_guard` of a wall / the box; new plan from the current estimate |
| `HALT: more than 3 replans` / `... from an obstacle (< r_guard)` | stopped for safety |
| `HALT: two looks in a row ... rejected by the gate` | tags disagree with the estimate → check perception / `tag_z_sign` / `world.yaml` |
| `=== done in ... s: ... drives, ... spins, ... looks, ... replans` | summary (also in `summary.yaml`) |

Tunables in `nav.yaml`:

| key | default | what |
|---|---|---|
| `plan: tie_side` | `right` | equal-length detours: `right` = below-left of the box (from S looking at G) |
| `plan: grid_res` / `simplify_tol` | `0.02` / `0.03` m | safe planner grid and path simplification |
| `exec: wp_spacing` | `0.5` m | waypoint spacing along the plan |
| `exec: pass_pos` / `pass_ang_deg` | `0.08` m / `15` | tolerance at intermediate waypoints |
| `exec: goal_pos` | `0.05` m | tolerance at G |
| `exec: goal_face_tag` / `goal_ang_deg` | `true` / `15` | end at G facing the nearest tag |
| `exec: guard_overshoot` / `max_replans` | `0.25` / `3` | collision check margin, replan limit |
| `ekf: relinearize` | `true` | iterated EKF update (large heading errors with close tags) |

`r_margin` (0.08 m) and `guard_margin` (0.03 m) are in `world.yaml` (`margins:`).

The terminal output of `hw4_nav` is also saved to `HW4/logs/run_<time>_<planner>/console.txt`.

## 7. Copy logs to the Mac

```bash
rsync -av ubuntu@<robot-ip>:~/ros2_ws/src/CSE276A/HW4/logs/ HW4/logs/
```

Per run: `config.yaml` (world + nav used), `plan.yaml` (path, waypoints, stats, replans),
`plan.npz` (planner data), `trajectory.csv`, `steps.csv`, `observations.csv`,
`waypoints.csv`, `summary.yaml`, `console.txt`.

## 8. End position and report

Write the measured end of each run (arena frame, robot centre):
```bash
cat > HW4/logs/run_<time>_safe/end.yaml <<'EOF'
{x: 2.51, y: 0.48}
EOF
cat > HW4/logs/run_<time>_fast/end.yaml <<'EOF'
{x: 2.49, y: 0.53}
EOF
```

Planner figures of the plan a run really used:
```bash
/opt/anaconda3/bin/python HW4/plot_plan.py --run HW4/logs/run_<time>_safe
/opt/anaconda3/bin/python HW4/plot_plan.py --run HW4/logs/run_<time>_fast
```
Outputs: `run_<time>_<planner>/figures/plan_<planner>.png`, `expand_<planner>.png`.

Report:
```bash
/opt/anaconda3/bin/python HW4/report_run.py      # newest safe + newest fast run
/opt/anaconda3/bin/python HW4/report_run.py HW4/logs/run_<time>_safe HW4/logs/run_<time>_fast
```
Outputs: per run `report.png` (plan, replans, estimated trajectory, measured end) and
`report.md`; for a safe + fast pair `HW4/figures/report_compare.png` and `HW4/report.md`
(planned vs driven length, time, drives / spins / looks, replans, planned / actual clearance
and body margin, end error estimated / measured, planner statistics).

## 9. Simulation (optional, Mac)

Closed loop with HW3-size motion errors (drives +20 %, veer 5°/m, spins −14 %), both planners,
10 seeds; checks no collision and the end error:
```bash
cd robot_control && /opt/anaconda3/bin/python -m pytest -q test/test_hw4_sim.py
```
