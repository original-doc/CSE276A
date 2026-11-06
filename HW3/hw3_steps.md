# HW3: EKF-SLAM demo — how to run

Branch `HW3`. 
One run = square × 3 laps → pause (measure) → filter reset → octagon × 3 laps.

| File | What |
|---|---|
| `robot_control/config/hw3/slam.yaml` | tolerances, levels, look, init limits, EKF noise, detector ids |
| `robot_control/config/hw3/apriltag.yaml` | apriltag node (ids 0–7, size 0.166) |
| `robot_control/launch/hw3_launch.py` | perception + `motor_control` + `hw3_slam` |
| `HW3/truth_map.yaml` | tape-measured tag poses (arena frame); evaluation only |
| `HW3/logs/run_<time>/{square,octagon}/` | logs per phase; you add `start.yaml` |
| `HW3/report_run.py` | figures + tables (Mac) |
| `HW3/sim_sweep.py`, `HW3/replay_hw2.py` | simulation sweep, HW2 log replay (Mac) |

## 0. Deploy

Robot:
```bash
cd ~/ros2_ws/src/CSE276A && git fetch && git checkout HW3 && git pull
cd ~/ros2_ws && colcon build --packages-select robot_control
source ~/ros2_ws/install/setup.bash        # in every new terminal
```

Rebuild after every pull, and after editing `apriltag.yaml` or launch files.
`slam.yaml`, calibration and logs are read from / written to the source folder (no rebuild).

Before every launch:
```bash
pgrep -af robot_vision_camera_node         # if a PID shows: kill -9 <pid>
```

## 1. Field setup

Arena 3 m × 3 m, origin = bottom-left corner, x along the bottom edge, y along the left edge.
Tag centres at camera height (~12.5 cm), faces pointing into the arena.

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

Measure every tag from the same corner, write the values into `HW3/truth_map.yaml`.

Start P: robot centre (base_link) at (1.189, 0.75), facing +x. Mark it with tape.

## 2. Perception check (no motion)

T1:
```bash
ros2 launch robot_control hw2_perception_launch.py \
  apriltag_config:=$HOME/ros2_ws/install/robot_control/share/robot_control/config/hw3/apriltag.yaml
```

T2:
```bash
ros2 topic hz /detections
ros2 run tf2_ros tf2_echo base_link tag_2
```

Noise stats (robot still, a tag at 0.5 / 1.0 / 1.5 / 2.0 / 2.5 m, also at 45–55° oblique):
```bash
ros2 run robot_control hw2_probe --ros-args -p mode:=stats -p duration_s:=10.0 \
  -p landmarks:=$HOME/ros2_ws/src/CSE276A/HW3/truth_map.yaml
```
Update `ekf: obs:` (a, b, c_deg, e_deg) in `slam.yaml` if needed.

Stop T1 before step 3.

## 3. Dry run (never moves)

Robot on P.
```bash
ros2 launch robot_control hw3_launch.py dry_run:=true
```
Expect the tags ahead (2, 3) added, with σ printed, and a planned first step.

Foxglove: `ws://192.168.138.130:8765`, 3D panel, fixed frame `map`;
topics `/hw3/pose`, `/hw3/landmarks`; image `/camera/image_raw/compressed`.

## 4. Short test

```bash
ros2 launch robot_control hw3_launch.py square_laps:=1 octagon_laps:=1 phase_pause_s:=10
```

## 5. Full run

1. Robot on P. Measure it; write later into `square/start.yaml`.
2. Run:
   ```bash
   ros2 launch robot_control hw3_launch.py
   ```
3. After the square phase it prints `Mark and measure the robot pose now ...` and counts down
   `phase_pause_s` (30 s). Tape-mark the robot centre and heading; measure it for `octagon/start.yaml`.
4. Octagon phase starts by itself. Everything shuts down at the end. Ctrl-C stops the motors.

Launch arguments (defaults):

| argument | default |
|---|---|
| `square_size` | `1.5` |
| `square_laps` / `octagon_laps` | `3` / `3` (`0` skips a phase) |
| `reset_between_phases` | `true` |
| `phase_pause_s` | `30.0` |
| `dry_run` | `false` |
| `floor` | `study_wood2_hw2_ext` |
| `tag_z_sign` | `1` |

Separate terminals instead (restart only the runner):
```bash
# T1
ros2 launch robot_control hw2_perception_launch.py \
  apriltag_config:=$HOME/ros2_ws/install/robot_control/share/robot_control/config/hw3/apriltag.yaml
# T2
ros2 run robot_control motor_control
# T3
ros2 run robot_control hw3_slam --ros-args -p square_laps:=3 -p octagon_laps:=3
```

`HALT: ...` = two looks in a row with every known tag gated → check perception / `tag_z_sign`.

The terminal output of `hw3_slam` is also saved to `HW3/logs/run_<time>/console.txt`.

## 6. Logs and report

Write the measured start of each phase (arena frame, robot centre):
```bash
cat > HW3/logs/run_<time>/square/start.yaml <<'EOF'
{x: 1.189, y: 0.75, theta_deg: 0.0}
EOF
cat > HW3/logs/run_<time>/octagon/start.yaml <<'EOF'
{x: 1.20, y: 0.74, theta_deg: 1.5}
EOF
```

Report:
```bash
python HW3/report_run.py HW3/logs/run_<time>     # no argument = newest run
```
Outputs: `run_<time>/report.md`; per phase `map.png`, `sigma.png`, `report.md`.

