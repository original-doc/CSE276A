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
