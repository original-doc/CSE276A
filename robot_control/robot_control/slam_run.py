"""HW3 run loop: phases of laps, each phase a fresh SLAM map (no ROS).

run_phases / run_goals / LapEnd are shared by the simulation and the node.
Hw3Session is everything hw3_slam does except ROS: logs, phases, the pause
between phases, lap tables and dry run; look / execute / sleep are injected.
"""
import math
import os
import time
from dataclasses import dataclass

from robot_control.ekf import noise_from_config, sigmas
from robot_control.estimator import SlamEstimator, init_limits_from_config
from robot_control.nav_log import SlamCsvLog, TeeLog, new_run_dir, write_yaml
from robot_control.paths import is_lap_end, make_phases
from robot_control.stop_look import Navigator, NavLog, next_move, params_from_config, to_step


@dataclass
class PhaseResult:
    name: str
    estimator: object
    results: list       # WaypointResult per goal


def run_phases(phases, new_estimator, run_phase, reset=True, between=None):
    """Run phases in order. reset: new estimator (new map frame, no landmarks) per phase.

    run_phase(phase, estimator) -> list of WaypointResult;
    between(previous_phase, next_phase) is called in between (pause, measure).
    """
    out, est = [], None
    for i, phase in enumerate(phases):
        if i and between is not None:
            between(phases[i - 1], phase)
        if est is None or reset:
            est = new_estimator()
        out.append(PhaseResult(phase.name, est, run_phase(phase, est)))
    return out


def run_goals(nav, goals, on_result=None):
    """All goals of one phase, goal index from 1."""
    results = []
    for i, goal in enumerate(goals, 1):
        results.append(nav.run_goal(i, goal))
        if on_result is not None:
            on_result(results[-1])
    return results


class LapEnd(NavLog):
    """Calls on_lap(lap, estimator) after the last goal of every lap."""

    def __init__(self, lap_index, estimator, on_lap):
        self.lap_index = lap_index
        self.estimator = estimator
        self.on_lap = on_lap

    def on_waypoint(self, result):
        if is_lap_end(self.lap_index, result.index):
            self.on_lap(self.lap_index[result.index - 1], self.estimator)


@dataclass(frozen=True)
class RunOptions:
    square_size: float = 1.5
    square_laps: int = 3
    octagon_laps: int = 3
    reset: bool = True
    phase_pause_s: float = 30.0
    wp_pause_s: float = 1.0
    countdown_s: int = 3
    dry_looks: int = 3


def map_table(est):
    """Landmarks of an estimator as printable lines (map frame)."""
    lines = [f'  {"id":>3} {"x":>7} {"y":>7} {"psi":>7}  {"sx":>6} {"sy":>6} {"spsi":>5}  upd']
    for lm in est.landmarks():
        c = est.landmark_cov(lm.id)
        lines.append(f'  {lm.id:>3} {lm.x:+7.3f} {lm.y:+7.3f} {math.degrees(lm.yaw):+7.1f}  '
                     f'{math.sqrt(c[0, 0]):6.3f} {math.sqrt(c[1, 1]):6.3f} '
                     f'{math.degrees(math.sqrt(c[2, 2])):5.1f}  {est.n_updates[lm.id]}')
    return lines


def fmt_pose(p):
    return f'({p.x:+.3f}, {p.y:+.3f}, {math.degrees(p.theta):+.1f} deg)'


def result_line(phase, lap, r):
    return (f'>>> {phase} lap {lap} vertex {r.index} '
            f'{"reached" if r.reached else "NOT reached"}: estimate {fmt_pose(r.pose)}, '
            f'{r.iters} looks, tags {r.tags_used} {r.note}')


class Hw3Session:
    """look() -> (accepted, rejected); execute(step) -> timing; extra_logs: e.g. publishers."""

    def __init__(self, cfg, calib, look, execute, opts, log_root, extra_logs=(),
                 run_config=None, sleep=time.sleep, out=print, clock=time.monotonic):
        self.cfg = cfg
        self.calib = calib
        self.params = params_from_config(cfg)
        self.noise = noise_from_config(cfg['ekf'])
        self.limits = init_limits_from_config(cfg['slam'])
        self.look = look
        self.execute = execute
        self.opts = opts
        self.log_root = log_root
        self.extra_logs = list(extra_logs)
        self.run_config = run_config or {}
        self.sleep = sleep
        self.out = out
        self.clock = clock
        self.run_dir = None

    def new_estimator(self):
        return SlamEstimator(self.noise, self.limits)

    def phases(self):
        o = self.opts
        return make_phases(o.square_size, o.square_laps, o.octagon_laps)

    def navigator(self, est, log):
        return Navigator(est, self.calib, self.params, self.look, self.execute,
                         TeeLog(log, *self.extra_logs))

    # ---------- full run ----------

    def run(self):
        """All phases; returns list[PhaseResult]. Logs go to <log_root>/run_<time>/."""
        self.run_dir = new_run_dir(self.log_root)
        os.makedirs(self.run_dir)
        write_yaml(os.path.join(self.run_dir, 'config.yaml'),
                   {**self.run_config, 'options': self.opts.__dict__, 'slam': self.cfg})
        self.out(f'log: {self.run_dir}')
        self.countdown(self.opts.countdown_s, 'starting in')
        return run_phases(self.phases(), self.new_estimator, self.run_phase, self.opts.reset,
                          self.between)

    def run_phase(self, phase, est):
        log = SlamCsvLog(os.path.join(self.run_dir, phase.name), est, phase.lap_index,
                         self.clock)
        lap_end = LapEnd(phase.lap_index, est, lambda lap, e: self.print_lap(phase, lap, e))
        nav = self.navigator(est, TeeLog(log, lap_end))
        self.out(f'=== phase {phase.name}: {len(phase.goals)} vertices, '
                 f'{phase.lap_index[-1]} laps; map frame = current pose; '
                 f'measure it into {os.path.join(log.dir, "start.yaml")}')
        try:
            return run_goals(nav, phase.goals, lambda r: self.after_goal(phase, r))
        finally:
            log.close()

    def after_goal(self, phase, r):
        self.out(result_line(phase.name, phase.lap_index[r.index - 1], r))
        self.sleep(self.opts.wp_pause_s)

    def print_lap(self, phase, lap, est):
        self.out(f'--- {phase.name} lap {lap} done: {len(est.ids)} landmarks')
        for line in map_table(est):
            self.out(line)

    def between(self, prev, nxt):
        path = os.path.join(self.run_dir, nxt.name, 'start.yaml')
        reset = 'reset: new map frame here' if self.opts.reset else 'no reset: same map'
        self.out(f'=== {prev.name} done. Mark and measure the robot pose now '
                 f'(write it into {path}); {reset}')
        self.countdown(int(self.opts.phase_pause_s), f'{nxt.name} starts in')

    def countdown(self, seconds, label):
        for i in range(seconds, 0, -1):
            self.out(f'{label} {i}...')
            self.sleep(1.0)

    # ---------- dry run ----------

    def dry_run(self):
        """Never moves: look a few times, print the map and the first planned step."""
        est = self.new_estimator()
        nav = self.navigator(est, NavLog())
        goal = self.phases()[0].goals[0]
        for _ in range(self.opts.dry_looks):
            nav.update_from_look(1)
            move = next_move(est.pose(), goal, self.params)
            plan = to_step(move, self.calib, self.params).describe() if move else 'reached'
            sx, sy, st = sigmas(est.pose_cov())
            self.out(f'estimate {fmt_pose(est.pose())} sigma ({sx:.3f}, {sy:.3f}, '
                     f'{math.degrees(st):.1f} deg); {len(est.ids)} landmarks -> {plan}')
        for line in map_table(est):
            self.out(line)
        return est
