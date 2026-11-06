import os

import numpy as np
import pytest
from sim_hw3 import SimClock, arena_map, centred_start, load_config
from sim_rover import SimRover, make_calib

from robot_control.slam_run import Hw3Session, RunOptions, map_table
from robot_control.stop_look import NavigationHalt


class Rig:
    """Sim rover + fake clock/sleep/print for a Hw3Session."""

    def __init__(self, tmp_path, opts, look=None, seed=0):
        self.rover = SimRover(arena_map(), centred_start(opts.square_size),
                              np.random.default_rng(seed))
        self.clock, self.lines, self.slept, self.executed = SimClock(), [], [], []
        self.session = Hw3Session(
            load_config(), make_calib(tmp_path), look or self.look, self.execute, opts,
            str(tmp_path / 'logs'), run_config={'floor': 'sim'}, sleep=self.sleep,
            out=self.lines.append, clock=self.clock)

    def look(self):
        self.clock.t += 1.0
        return self.rover.look()

    def execute(self, step):
        self.executed.append(step)
        t0 = self.clock.t
        self.clock.t += step.T
        self.rover.execute(step)
        return t0, self.clock.t

    def sleep(self, s):
        self.slept.append(s)
        self.clock.t += s


OPTS = RunOptions(square_laps=1, octagon_laps=1, phase_pause_s=5.0, wp_pause_s=0.5)


def test_run_writes_both_phases_and_pauses_between(tmp_path):
    rig = Rig(tmp_path, OPTS)
    out = rig.session.run()
    run_dir = rig.session.run_dir
    assert sorted(os.listdir(run_dir)) == ['config.yaml', 'console.txt', 'octagon', 'square']
    for phase in ('square', 'octagon'):
        files = os.listdir(os.path.join(run_dir, phase))
        assert 'map_lap1.yaml' in files and 'final_state.npz' in files
    assert [r.name for r in out] == ['square', 'octagon']
    assert out[0].estimator is not out[1].estimator                 # reset
    assert all(r.reached for p in out for r in p.results)
    n_goals = 5 + 8
    assert rig.slept == [1.0] * 3 + [0.5] * 5 + [1.0] * 5 + [0.5] * 8
    assert len(rig.slept) == 3 + n_goals + 5
    text = '\n'.join(rig.lines)
    assert 'square lap 1 done: 8 landmarks' in text and 'octagon lap 1 done' in text
    assert 'octagon starts in 5...' in text and 'octagon/start.yaml' in text


def test_no_reset_keeps_estimator(tmp_path):
    rig = Rig(tmp_path, RunOptions(square_laps=1, octagon_laps=1, reset=False,
                                   phase_pause_s=0, wp_pause_s=0))
    out = rig.session.run()
    assert out[0].estimator is out[1].estimator
    assert 'no reset' in '\n'.join(rig.lines)


def test_dry_run_never_moves(tmp_path):
    rig = Rig(tmp_path, OPTS)
    est = rig.session.dry_run()
    assert rig.executed == [] and rig.session.run_dir is None
    assert len(est.ids) >= 2                    # tags 2, 3 ahead of P
    assert sum('landmarks ->' in line for line in rig.lines) == OPTS.dry_looks
    assert len(map_table(est)) == len(est.ids) + 1


def test_halt_still_closes_the_phase_log(tmp_path):
    rig = Rig(tmp_path, OPTS)
    calls = []

    def look():
        calls.append(1)
        seen, rej = rig.look()
        if len(calls) > 1:                      # after the first look every tag jumps 0.5 m
            seen = {i: o.__class__(o.id, o.stamp, o.x + 0.5, o.y, o.psi, o.dist, o.oblique,
                                   o.count) for i, o in seen.items()}
        return seen, rej

    rig.session.look = look
    with pytest.raises(NavigationHalt):
        rig.session.run()
    assert 'final_state.npz' in os.listdir(os.path.join(rig.session.run_dir, 'square'))
    with open(os.path.join(rig.session.run_dir, 'console.txt')) as f:
        console = f.read()
    assert console.splitlines()[-1].startswith('HALT: ')


def test_console_has_the_printed_lines(tmp_path):
    rig = Rig(tmp_path, OPTS)
    rig.session.run()
    with open(os.path.join(rig.session.run_dir, 'console.txt')) as f:
        console = f.read().splitlines()
    assert console == rig.lines
    assert any(line.startswith('>>> square lap 1 vertex 5') for line in console)
