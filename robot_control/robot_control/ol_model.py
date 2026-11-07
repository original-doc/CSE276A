"""Open-loop calibration model (no ROS).

Per command level c, a primitive run for T seconds moves
    drive: d(T) = v * T + d0      (m)
    spin:  a(T) = w * T + a0      (deg, per direction)
Fits are always recomputed from the raw trials in config/open_loop/<floor>.yaml.

A spin level may instead carry a `pulse` model (spin: <dir>: pulse: {<cmd>: {...}}):
the same line a = rate * T + offset, but on the COMMANDED T, fitted from the heading
corrections of closed-loop runs (robot_control/fit_spin_pulse.py) and valid down to `min_T`.
It replaces the trial fit and `time_scale` for that level, so one coarse level covers
every turn from a few degrees up (no gap between a fine and a coarse level).
"""
import math
import os
from dataclasses import asdict, dataclass
from datetime import date, datetime

import yaml

DIRS = ('ccw', 'cw')
MIN_T = 0.2      # s, shorter runs are unreliable
EXTRAP = 0.2     # allowed extrapolation beyond calibrated T range (fraction)
DEFAULT_CALIB_DIR = '~/ros2_ws/src/CSE276A/robot_control/config/open_loop'


class ModelError(ValueError):
    pass


@dataclass
class LineFit:
    rate: float         # m/s or deg/s
    offset: float       # m or deg
    rms: float          # residual rms
    n: int              # trials used
    n_durations: int    # distinct T values (1 -> offset forced to 0)
    t_min: float
    t_max: float

    def at(self, T):
        return self.rate * T + self.offset


def fit_line(points):
    """Least-squares value = rate * T + offset over [(T, value)]."""
    if not points:
        raise ModelError('no trials')
    n = len(points)
    Ts = [p[0] for p in points]
    vs = [p[1] for p in points]
    distinct = len(set(Ts))
    if distinct == 1:
        rate, offset = sum(vs) / n / Ts[0], 0.0
    else:
        mt, mv = sum(Ts) / n, sum(vs) / n
        stt = sum((t - mt) ** 2 for t in Ts)
        rate = sum((t - mt) * (v - mv) for t, v in points) / stt
        offset = mv - rate * mt
    rms = math.sqrt(sum((v - (rate * t + offset)) ** 2 for t, v in points) / n)
    return LineFit(rate, offset, rms, n, distinct, min(Ts), max(Ts))


@dataclass
class PulseModel:
    """Single pulses of one spin level: deg = rate * T + offset on the commanded T."""
    rate: float             # deg/s
    offset: float           # deg; negative = dead time before the robot starts turning
    min_T: float            # s, shortest pulse allowed
    t_min: float            # s, range of the fitted data
    t_max: float
    rms: float = 0.0        # deg, residual of the fit
    n: int = 0              # samples fitted

    def at(self, T):
        return self.rate * T + self.offset

    @property
    def min_deg(self):
        """Smallest turn this level can make (one pulse of min_T)."""
        return self.at(self.min_T)

    def time(self, deg):
        """Pulse length for `deg`; raises ModelError below min_T. Returns (T, warning)."""
        if self.rate <= 0:
            raise ModelError(f'non-positive pulse rate {self.rate:.3f}')
        T = (deg - self.offset) / self.rate
        if T < self.min_T:
            raise ModelError(f'pulse T = {T:.2f} s < {self.min_T} s for {deg:g} deg '
                             f'(smallest {self.min_deg:.1f} deg)')
        warn = None
        if T < self.t_min * (1 - EXTRAP) or T > self.t_max * (1 + EXTRAP):
            warn = f'pulse T = {T:.2f} s outside fitted {self.t_min:g}-{self.t_max:g} s'
        return T, warn

    def to_yaml(self):
        """Plain python numbers (yaml.safe_dump cannot write numpy floats)."""
        return {k: (round(float(v), 4) if k != 'n' else int(v)) for k, v in asdict(self).items()}


def pulse_from_yaml(d):
    return PulseModel(float(d['rate']), float(d['offset']), float(d['min_T']),
                      float(d['t_min']), float(d['t_max']), float(d.get('rms', 0.0)),
                      int(d.get('n', 0)))


def fit_for(fits, cmd):
    """Fit of the calibrated level cmd (exact match only)."""
    for k, f in fits.items():
        if abs(k - cmd) < 1e-9:
            return f
    raise ModelError(f'cmd {cmd} not calibrated (levels: {sorted(fits)})')


def invert(fit, target, min_T=MIN_T):
    """Run time T reaching target. Returns (T, warning or None)."""
    if fit.rate <= 0:
        raise ModelError(f'non-positive rate {fit.rate:.3f}')
    T = (target - fit.offset) / fit.rate
    if T < min_T:
        raise ModelError(f'T = {T:.2f} s < {min_T} s for target {target:g}')
    warn = None
    if T < fit.t_min * (1 - EXTRAP) or T > fit.t_max * (1 + EXTRAP):
        warn = f'T = {T:.2f} s outside calibrated {fit.t_min:g}-{fit.t_max:g} s'
    elif fit.n_durations < 2:
        warn = 'only one calibrated duration (offset assumed 0)'
    return T, warn


def wrap_deg(a):
    """Wrap to (-180, 180]."""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def now_str():
    return datetime.now().isoformat(timespec='seconds')


class Calibration:
    def __init__(self, data, path=None):
        self.data = data
        self.path = path
        data.setdefault('drive', {}).setdefault('trials', [])
        for d in DIRS:
            data.setdefault('spin', {}).setdefault(d, {}).setdefault('trials', [])
        data.setdefault('verify', [])

    @classmethod
    def load(cls, path, floor=None, create=False):
        path = os.path.expanduser(path)
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        elif create:
            data = {'floor': floor}
        else:
            raise ModelError(f'calibration file not found: {path}')
        return cls(data, path)

    @classmethod
    def for_floor(cls, calib_dir, floor, create=False):
        if not floor:
            raise ModelError('Set the floor: -p floor:=<name> (e.g. wood).')
        return cls.load(os.path.join(os.path.expanduser(calib_dir), f'{floor}.yaml'),
                        floor, create)

    def save(self):
        self.data['updated'] = date.today().isoformat()
        try:
            self.data['fit'] = self.summary()
        except ModelError:
            pass
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump(self.data, f, sort_keys=False, default_flow_style=None)
        os.replace(tmp, self.path)

    # ---------- data ----------

    def drive_trials(self):
        return self.data['drive']['trials']

    def spin_trials(self, d):
        return self.data['spin'][d]['trials']

    def add(self, trials, trial):
        """Append a trial (stamped) and save immediately."""
        trial['time'] = now_str()
        trials.append(trial)
        self.save()

    # ---------- fits ----------

    def drive_fits(self):
        """{cmd: LineFit} of distance travelled (chord length) vs T."""
        groups = {}
        for t in self.drive_trials():
            d = math.hypot(t['x'], t.get('y', 0.0))
            groups.setdefault(float(t['cmd']), []).append((float(t['T']), d))
        return {c: fit_line(p) for c, p in groups.items()}

    def spin_fits(self, d):
        groups = {}
        for t in self.spin_trials(d):
            groups.setdefault(float(t['cmd']), []).append((float(t['T']), abs(t['angle'])))
        return {c: fit_line(p) for c, p in groups.items()}

    def min_T(self, section, cmd):
        """Shortest allowed run for one level: section['min_T'][cmd], else MIN_T."""
        for k, v in (section.get('min_T') or {}).items():
            if abs(float(k) - cmd) < 1e-9:
                return float(v)
        return MIN_T

    def drive_time(self, dist, cmd):
        return invert(fit_for(self.drive_fits(), cmd), dist, self.min_T(self.data['drive'], cmd))

    def pulse(self, d, cmd):
        """PulseModel of spin level cmd in direction d, or None if the file has none."""
        for k, v in (self.data['spin'][d].get('pulse') or {}).items():
            if abs(float(k) - cmd) < 1e-9:
                return pulse_from_yaml(v)
        return None

    def set_pulse(self, d, cmd, model):
        self.data['spin'][d].setdefault('pulse', {})[cmd] = model.to_yaml()

    def spin_time(self, d, deg, cmd):
        """(T, warning) for one spin; the pulse model wins when the level has one."""
        pm = self.pulse(d, cmd)
        if pm is not None:
            return pm.time(deg)
        fit = fit_for(self.spin_fits(d), cmd)
        min_T = self.min_T(self.data['spin'][d], cmd)
        T, warn = invert(fit, deg, min_T)
        # Optional direction-specific correction to command duration. Keep raw
        # trial angles intact so the measured fit remains auditable.
        scale = float(self.data['spin'][d].get('time_scale', 1.0))
        if not math.isfinite(scale) or scale <= 0:
            raise ModelError(f'spin {d} time_scale must be positive and finite')
        T *= scale
        if T < min_T:
            raise ModelError(f'adjusted T = {T:.2f} s < {min_T} s for target {deg:g}')
        if T < fit.t_min * (1 - EXTRAP) or T > fit.t_max * (1 + EXTRAP):
            warn = f'adjusted T = {T:.2f} s outside calibrated {fit.t_min:g}-{fit.t_max:g} s'
        return T, warn

    def summary(self):
        def dump(fits):
            return {c: {k: round(v, 4) if isinstance(v, float) else v
                        for k, v in asdict(f).items()} for c, f in sorted(fits.items())}
        return {'drive': dump(self.drive_fits()),
                'spin': {d: dump(self.spin_fits(d)) for d in DIRS}}


def format_fit(name, cmd, f, unit):
    note = '' if f.n_durations >= 2 else '  (1 duration, offset=0)'
    return (f'{name} cmd {cmd:g}: rate {f.rate:.4f} {unit}/s, offset {f.offset:+.4f} {unit}, '
            f'rms {f.rms:.4f} {unit}, n={f.n}, T {f.t_min:g}-{f.t_max:g} s{note}')
