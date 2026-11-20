import importlib.util
import os

import pytest

PLOT_PLAN = os.path.join(os.path.dirname(__file__), '..', '..', 'HW4', 'plot_plan.py')
spec = importlib.util.spec_from_file_location('plot_plan', PLOT_PLAN)
plot_plan = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot_plan)


def test_make_all_figures(tmp_path):
    plot_plan.main(['--out', str(tmp_path)])
    names = ['plan_safe.png', 'plan_fast.png', 'plan_compare.png',
             'expand_safe.png', 'expand_fast.png']
    for name in names:
        assert (tmp_path / name).stat().st_size > 10_000


def test_box_clearance():
    world = plot_plan.load_world(plot_plan.DEFAULT_WORLD)
    fast = plot_plan.plan_fast(world)
    assert plot_plan.box_clearance(fast, world) == pytest.approx(fast.min_clearance)


@pytest.mark.parametrize('kind', ['safe', 'fast'])
def test_redraw_from_run(tmp_path, kind):
    from test_plan_run import Rig
    s = Rig(tmp_path, kind).session.run()
    world, plan = plot_plan.load_run_plan(s.run_dir)
    assert plan.kind == kind
    assert plan.length == pytest.approx(s.plans[0].length, abs=1e-3)
    plot_plan.main(['--run', s.run_dir])
    for name in (f'plan_{kind}.png', f'expand_{kind}.png'):
        assert os.path.getsize(os.path.join(s.run_dir, 'figures', name)) > 10_000
