from robot_control.paths import Phase, is_lap_end
from robot_control.slam_run import LapEnd, run_goals, run_phases
from robot_control.stop_look import WaypointResult


def test_is_lap_end():
    idx = [1, 1, 2, 2, 2]
    assert [is_lap_end(idx, i) for i in range(1, 6)] == [False, True, False, False, True]


def phases():
    return [Phase('square', ['a', 'b'], [1, 2]), Phase('octagon', ['c'], [1])]


def test_run_phases_order_reset_and_between():
    calls = []
    made = []

    def new_estimator():
        made.append(object())
        calls.append('new')
        return made[-1]

    def run_phase(phase, est):
        calls.append((phase.name, made.index(est)))
        return [phase.name]

    out = run_phases(phases(), new_estimator, run_phase,
                     between=lambda a, b: calls.append(('pause', a.name, b.name)))
    assert calls == ['new', ('square', 0), ('pause', 'square', 'octagon'), 'new',
                     ('octagon', 1)]
    assert [(r.name, r.results) for r in out] == [
        ('square', ['square']), ('octagon', ['octagon'])]
    made.clear()
    out = run_phases(phases(), new_estimator, run_phase, reset=False)
    assert len(made) == 1 and out[0].estimator is out[1].estimator


class FakeNav:
    def run_goal(self, i, goal):
        return WaypointResult(i, goal, True, True, goal, (0, 0, 0), 1)


def test_run_goals_and_lap_end():
    laps = []
    end = LapEnd([1, 1, 2], 'est', lambda lap, e: laps.append((lap, e)))
    results = run_goals(FakeNav(), ['a', 'b', 'c'], end.on_waypoint)
    assert [r.index for r in results] == [1, 2, 3]
    assert laps == [(1, 'est'), (2, 'est')]
