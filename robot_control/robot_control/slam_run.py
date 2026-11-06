"""HW3 run loop: phases of laps, each phase a fresh SLAM map (no ROS).

The ROS node and the simulation share this; they supply how to build an
estimator and how to run one phase (log, navigator, goals).
"""
from dataclasses import dataclass

from robot_control.paths import is_lap_end
from robot_control.stop_look import NavLog


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
