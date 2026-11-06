import importlib.util
import math
import os

import yaml

HERE = os.path.dirname(__file__)
spec = importlib.util.spec_from_file_location(
    'replay_hw2', os.path.join(HERE, '..', '..', 'HW3', 'replay_hw2.py'))
replay_hw2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay_hw2)

RUN = str(replay_hw2.DEFAULT_RUN)


def slam_cfg():
    with open(replay_hw2.DEFAULT_SLAM) as f:
        return yaml.safe_load(f)


def test_events_in_time_order_look_before_step():
    events = replay_hw2.load_events(RUN)
    assert [e.kind for e in events[:3]] == ['look', 'step', 'look']     # t 4.01: look, then drive
    assert events[1].data.kind == 'drive' and events[1].data.value == 0.89
    assert all(a.t <= b.t for a, b in zip(events, events[1:]))
    spins = [e.data for e in events if e.kind == 'step' and e.data.kind == 'spin']
    assert spins[0].value > 0 and spins[1].value < 0           # ccw 88.7, then cw 6.7
    looks = [e for e in events if e.kind == 'look']
    assert sum(len(e.data) for e in looks) == 13     # the row without values is dropped


def test_replay_maps_three_tags_without_the_map():
    est = replay_hw2.make_estimator(slam_cfg())
    history = replay_hw2.replay(replay_hw2.load_events(RUN), est)
    assert est.ids == [0, 1, 2]
    actions = [(r.obs.id, r.action) for _, recs, _ in history for r in recs or []]
    assert actions[0] == (0, 'init')                            # HW2 gated it; SLAM adds it
    assert actions.count((1, 'skip_init')) == 2                 # 2.7 m > init_max_dist
    assert all(a != 'gate' for _, a in actions)
    end = replay_hw2.pose_at(history, 58.81)
    assert math.hypot(end.x, end.y) < 0.1                       # back near the start


def test_report_renders():
    text = replay_hw2.report(RUN, slam_cfg(), replay_hw2.map_eval.truth_poses(
        replay_hw2.load_map(replay_hw2.DEFAULT_MAP)))
    assert 'after best rigid fit' in text and text.count('| update') >= 9
