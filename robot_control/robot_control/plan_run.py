"""HW4 run loop: plan, then follow the plan with stop-look-correct (no ROS).

PlanSession is everything hw4_nav does except ROS: plan, logs, waypoints with
per-waypoint tolerances, the drive guard and re-planning, dry run. look /
execute / sleep are injected (the simulation uses the same class).
"""
import dataclasses
import math
import os
import time
from collections import Counter
from dataclasses import dataclass, field

from robot_control.ekf import noise_from_config, sigmas
from robot_control.estimator import LocalizationEstimator
from robot_control.nav_log import (
    CsvNavLog, new_run_dir, plan_record, TeeLog, write_plan_npz, write_yaml)
from robot_control.path_utils import densify, NoPath, to_goals
from robot_control.plan_fast import plan_fast
from robot_control.plan_safe import plan_safe
from robot_control.stop_look import (
    NavigationHalt, Navigator, NavLog, next_move, params_from_config, Replan, to_step)
from robot_control.world import clearance, parse_world, segment_clearance

PLANNERS = {'safe': plan_safe, 'fast': plan_fast}


@dataclass(frozen=True)
class ExecOptions:
    wp_spacing: float
    pass_pos: float
    pass_ang: float         # rad
    goal_pos: float
    guard_overshoot: float
    max_replans: int

    @classmethod
    def from_config(cls, e):
        return cls(float(e['wp_spacing']), float(e['pass_pos']),
                   math.radians(float(e['pass_ang_deg'])), float(e['goal_pos']),
                   float(e['guard_overshoot']), int(e['max_replans']))


@dataclass(frozen=True)
class RunOptions:
    countdown_s: int = 3
    wp_pause_s: float = 1.0
    dry_looks: int = 3


@dataclass
class RunSummary:
    run_dir: str
    plans: list             # [Plan]: the first one, then one per replan
    results: list           # [WaypointResult]
    replans: int
    elapsed_s: float
    counts: dict = field(default_factory=dict)  # drive / spin / look


def drive_guard(world, r_guard, overshoot):
    """guard(pose, move): False if the drive, run `overshoot` long, passes within r_guard."""
    def guard(pose, move):
        d = move.value * (1.0 + overshoot)
        end = (pose.x + d * math.cos(pose.theta), pose.y + d * math.sin(pose.theta))
        return segment_clearance((pose.x, pose.y), end, world) >= r_guard
    return guard


def goal_params(params, ex, last):
    """Intermediate waypoints: pass tolerances; the goal: tight position, any heading."""
    if last:
        return dataclasses.replace(params, pos_tol=ex.goal_pos, ang_tol=math.pi)
    return dataclasses.replace(params, pos_tol=ex.pass_pos, ang_tol=ex.pass_ang)


def plan_from(world, kind, plan_cfg, start):
    """Plan with r_inflate; if the start is not free for it, retry with r_guard.

    Returns (plan, used_r_guard). Raises NavigationHalt if neither works.
    """
    planner = PLANNERS[kind]
    try:
        return planner(world, plan_cfg, start=start), False
    except NoPath:
        pass
    c = clearance(start, world)
    if c < world.r_guard:
        raise NavigationHalt(f'robot at {start} is {c:.3f} m from an obstacle '
                             f'(< r_guard {world.r_guard:.3f}): stopped')
    try:
        return planner(world, plan_cfg, start=start, r=world.r_guard), True
    except NoPath as e:
        raise NavigationHalt(f'no path from {start}: {e}')


def plan_goals(plan, ex):
    return to_goals(densify(plan.points, ex.wp_spacing))


class ActionCount(NavLog):
    def __init__(self):
        self.counts = Counter()

    def on_look(self, goal_index, records, estimator):
        self.counts['look'] += 1

    def on_step(self, goal_index, step, *args, **kwargs):
        self.counts[step.kind] += 1


def fmt_pose(p):
    return f'({p.x:+.3f}, {p.y:+.3f}, {math.degrees(p.theta):+.1f} deg)'


class PlanSession:
    """look() -> (accepted, rejected); execute(step) -> timing; on_plan(plan): e.g. publish.

    world_data = world.yaml contents (saved into the run's config.yaml).
    """

    def __init__(self, cfg, world_data, kind, calib, look, execute, log_root,
                 opts=RunOptions(), extra_logs=(), run_config=None, on_plan=None,
                 guard=None, sleep=time.sleep, out=print, clock=time.monotonic):
        if kind not in PLANNERS:
            raise ValueError(f'planner must be one of {sorted(PLANNERS)}, not {kind!r}')
        self.cfg = cfg
        self.world_data = world_data
        self.world = parse_world(world_data)
        self.kind = kind
        self.calib = calib
        self.params = params_from_config(cfg)
        self.noise = noise_from_config(cfg['ekf'])
        self.ex = ExecOptions.from_config(cfg['exec'])
        self.plan_cfg = cfg['plan']
        self.look = look
        self.execute = execute
        self.log_root = log_root
        self.opts = opts
        self.extra_logs = list(extra_logs)
        self.run_config = run_config or {}
        self.on_plan = on_plan or (lambda plan: None)
        self.guard = guard or drive_guard(self.world, self.world.r_guard,
                                          self.ex.guard_overshoot)
        self.sleep = sleep
        self.out = out
        self.clock = clock
        self.run_dir = None
        self.nav = None
        self.console = None

    def start_xy(self):
        return (self.world.start.x, self.world.start.y)

    def new_estimator(self):
        return LocalizationEstimator(self.world.landmarks, self.world.start, self.noise)

    def navigator(self, est, log):
        return Navigator(est, self.calib, self.params, self.look, self.execute,
                         TeeLog(log, *self.extra_logs), self.guard)

    def plan(self, start):
        plan, used_guard = plan_from(self.world, self.kind, self.plan_cfg, start)
        self.on_plan(plan)
        return plan, used_guard

    def say(self, text):
        self.out(text)
        if self.console is not None:
            self.console.write(text + '\n')
            self.console.flush()

    def print_plan(self, plan, goals, used_guard):
        st = plan.stats
        r = 'r_guard' if used_guard else 'r_inflate'
        self.say(f'=== {plan.kind} plan ({r} = {st["r"]:.3f} m): length {plan.length:.3f} m, '
                 f'min clearance {plan.min_clearance:.3f} m, {st["nodes"]} nodes, '
                 f'{st["expanded"]} expanded, {st["time_s"] * 1000:.0f} ms')
        for i, g in enumerate(goals, 1):
            self.say(f'  wp {i:2d}: ({g.x:.3f}, {g.y:.3f}) heading {math.degrees(g.theta):+.1f}')

    # ---------- full run ----------

    def run(self):
        """Plan, then drive to the goal; logs in <log_root>/run_<time>_<kind>/."""
        self.run_dir = new_run_dir(self.log_root) + f'_{self.kind}'
        os.makedirs(self.run_dir)
        write_yaml(os.path.join(self.run_dir, 'config.yaml'),
                   {**self.run_config, 'planner': self.kind, 'world': self.world_data,
                    'nav': self.cfg})
        self.console = open(os.path.join(self.run_dir, 'console.txt'), 'w')
        try:
            self.say(f'log: {self.run_dir}')
            return self.drive()
        except NavigationHalt as e:
            self.say(f'HALT: {e}')
            raise
        except KeyboardInterrupt:
            self.say('Aborted (Ctrl-C).')
            raise
        finally:
            self.console.close()
            self.console = None

    def drive(self):
        plan, used_guard = self.plan(self.start_xy())
        goals = plan_goals(plan, self.ex)
        self.print_plan(plan, goals, used_guard)
        record = {**plan_record(plan, goals), 'used_r_guard': used_guard, 'replans': []}
        write_yaml(os.path.join(self.run_dir, 'plan.yaml'), record)
        write_plan_npz(os.path.join(self.run_dir, 'plan.npz'), plan)

        est = self.new_estimator()
        log = CsvNavLog(self.run_dir, self.world.start, sigmas(est.pose_cov()), self.clock)
        count = ActionCount()
        self.nav = self.navigator(est, TeeLog(log, count))
        self.countdown(self.opts.countdown_s)
        t0 = self.clock()
        plans, results, replans, index = [plan], [], 0, 0
        try:
            while True:
                try:
                    index = self.follow(goals, index, results)
                    break
                except Replan as e:
                    replans += 1
                    self.say(f'>>> REPLAN {replans}: {e}')
                    if replans > self.ex.max_replans:
                        raise NavigationHalt(f'more than {self.ex.max_replans} replans')
                    start = (e.pose.x, e.pose.y)
                    plan, used_guard = self.plan(start)
                    goals = plan_goals(plan, self.ex)
                    plans.append(plan)
                    self.print_plan(plan, goals, used_guard)
                    record['replans'].append({
                        't': round(self.clock() - t0, 2), 'pose': [
                            round(e.pose.x, 4), round(e.pose.y, 4),
                            round(math.degrees(e.pose.theta), 2)],
                        'used_r_guard': used_guard, **plan_record(plan, goals)})
                    write_yaml(os.path.join(self.run_dir, 'plan.yaml'), record)
        finally:
            log.close()
        summary = RunSummary(self.run_dir, plans, results, replans, self.clock() - t0,
                             dict(count.counts))
        self.print_summary(summary, est)
        return summary

    def follow(self, goals, index, results):
        """Run goals in order; returns the last waypoint index (indices go on across replans)."""
        for k, goal in enumerate(goals):
            index += 1
            self.nav.p = goal_params(self.params, self.ex, k == len(goals) - 1)
            r = self.nav.run_goal(index, goal)
            results.append(r)
            self.say(f'>>> wp {index} ({goal.x:.3f}, {goal.y:.3f}) '
                     f'{"reached" if r.reached else "NOT reached"}: estimate '
                     f'{fmt_pose(r.pose)}, {r.iters} looks, tags {r.tags_used} {r.note}')
            self.sleep(self.opts.wp_pause_s)
        return index

    def print_summary(self, s, est):
        c = s.counts
        self.say(f'=== done in {s.elapsed_s:.1f} s: {c.get("drive", 0)} drives, '
                 f'{c.get("spin", 0)} spins, {c.get("look", 0)} looks, {s.replans} replans; '
                 f'final estimate {fmt_pose(est.pose())}; log {self.run_dir}')

    def countdown(self, seconds):
        for i in range(seconds, 0, -1):
            self.say(f'starting in {i}...')
            self.sleep(1.0)

    # ---------- dry run ----------

    def dry_run(self):
        """Never moves: plan, look a few times, print the estimate and the first step."""
        plan, used_guard = self.plan(self.start_xy())
        goals = plan_goals(plan, self.ex)
        self.print_plan(plan, goals, used_guard)
        est = self.new_estimator()
        nav = self.navigator(est, NavLog())
        for _ in range(self.opts.dry_looks):
            nav.update_from_look(1)
            move = next_move(est.pose(), goals[0], goal_params(self.params, self.ex, False))
            step = to_step(move, self.calib, self.params).describe() if move else 'reached'
            sx, sy, st = sigmas(est.pose_cov())
            self.say(f'estimate {fmt_pose(est.pose())} sigma ({sx:.3f}, {sy:.3f}, '
                     f'{math.degrees(st):.1f} deg) -> {step}')
        return plan, est
