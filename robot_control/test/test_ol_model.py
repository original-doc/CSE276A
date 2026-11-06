import pytest

from robot_control.ol_model import (
    Calibration, ModelError, PulseModel, fit_for, fit_line, invert, pulse_from_yaml, wrap_deg,
)


def make_calib(tmp_path):
    c = Calibration.load(tmp_path / 'wood.yaml', 'wood', create=True)
    for T in (2.0, 3.0, 4.0):
        for cmd, v, d0 in ((0.2, 0.25, -0.03), (0.4, 0.5, 0.05)):
            c.drive_trials().append({'cmd': cmd, 'T': T, 'x': v * T + d0, 'y': 0.0})
    for T in (0.5, 1.5):
        c.spin_trials('ccw').append({'cmd': 0.35, 'T': T, 'angle': 120.0 * T + 6.0})
    return c


def test_fit_known_line():
    f = fit_line([(T, 0.375 * T - 0.02) for T in (2.0, 3.0, 4.0, 5.0)])
    assert f.rate == pytest.approx(0.375)
    assert f.offset == pytest.approx(-0.02)
    assert f.rms == pytest.approx(0.0, abs=1e-12)
    assert (f.n, f.n_durations, f.t_min, f.t_max) == (4, 4, 2.0, 5.0)


def test_fit_single_duration_zero_offset():
    f = fit_line([(4.0, 1.5), (4.0, 1.46)])
    assert f.offset == 0.0
    assert f.rate == pytest.approx(1.48 / 4.0)
    assert f.rms == pytest.approx(0.02)


def test_fit_for_exact_level(tmp_path):
    fits = make_calib(tmp_path).drive_fits()
    assert fit_for(fits, 0.4).rate == pytest.approx(0.5)
    assert fit_for(fits, 0.2).offset == pytest.approx(-0.03)
    with pytest.raises(ModelError):
        fit_for(fits, 0.3)              # no interpolation
    with pytest.raises(ModelError):
        fit_for({}, 0.3)


def test_invert(tmp_path):
    c = make_calib(tmp_path)
    T, warn = c.drive_time(1.0, 0.4)
    assert T == pytest.approx((1.0 - 0.05) / 0.5)
    assert warn is None
    T, warn = c.spin_time('ccw', 90.0, 0.35)
    assert T == pytest.approx(84.0 / 120.0)
    assert warn is None


def test_invert_warnings_and_errors(tmp_path):
    c = make_calib(tmp_path)
    _, warn = c.drive_time(4.0, 0.4)    # T = 7.9 s, calibrated up to 4 s
    assert 'outside' in warn
    _, warn = c.drive_time(2.35, 0.4)   # T = 4.6 s, within +20%
    assert warn is None
    with pytest.raises(ModelError):
        c.drive_time(0.1, 0.4)          # T < 0.2 s
    with pytest.raises(ModelError):
        c.drive_time(1.0, 0.3)          # level not calibrated
    with pytest.raises(ModelError):
        c.spin_time('cw', 90.0, 0.35)   # not calibrated
    single = fit_line([(4.0, 1.5)])
    assert 'one calibrated duration' in invert(single, 1.4)[1]


def test_time_scale(tmp_path):
    c = make_calib(tmp_path)
    c.data['spin']['ccw']['time_scale'] = 1.15
    T, _ = c.spin_time('ccw', 90.0, 0.35)
    assert T == pytest.approx(84.0 / 120.0 * 1.15)
    c.data['spin']['ccw']['time_scale'] = 0
    with pytest.raises(ModelError):
        c.spin_time('ccw', 90.0, 0.35)


def test_yaml_roundtrip(tmp_path):
    c = make_calib(tmp_path)
    c.add(c.spin_trials('cw'), {'cmd': 0.35, 'T': 1.0, 'angle': 118.0})
    d = Calibration.load(tmp_path / 'wood.yaml')
    assert d.data['floor'] == 'wood'
    assert len(d.drive_trials()) == 6
    assert d.spin_trials('cw')[0]['angle'] == 118.0
    assert 'time' in d.spin_trials('cw')[0]
    assert d.drive_fits()[0.4].rate == pytest.approx(0.5)
    assert 'fit' in d.data
    with pytest.raises(ModelError):
        Calibration.load(tmp_path / 'missing.yaml')
    with pytest.raises(ModelError):
        Calibration.for_floor(tmp_path, '')


def test_wrap_deg():
    assert wrap_deg(190) == -170
    assert wrap_deg(-180) == 180
    assert wrap_deg(540) == 180


def test_pulse_model():
    pm = PulseModel(179.6, -18.6, 0.15, 0.22, 0.58, 2.3, 6)
    assert pm.min_deg == pytest.approx(179.6 * 0.15 - 18.6)
    T, warn = pm.time(30.0)
    assert T == pytest.approx((30.0 + 18.6) / 179.6) and warn is None
    _, warn = pm.time(10.0)                 # 0.16 s < 0.22 s - 20 %
    assert 'outside fitted' in warn
    with pytest.raises(ModelError):
        pm.time(5.0)                        # below min_T
    with pytest.raises(ModelError):
        PulseModel(0.0, 0.0, 0.1, 0.1, 1.0).time(10.0)
    assert pulse_from_yaml(pm.to_yaml()) == pm


def test_spin_time_prefers_the_pulse_model_and_roundtrips(tmp_path):
    c = make_calib(tmp_path)
    c.data['spin']['ccw']['time_scale'] = 1.15
    c.set_pulse('ccw', 0.35, PulseModel(136.9, -6.3, 0.1, 0.25, 0.86, 4.4, 49))
    T, warn = c.spin_time('ccw', 90.0, 0.35)
    assert T == pytest.approx((90.0 + 6.3) / 136.9) and warn is None     # no time_scale
    with pytest.raises(ModelError):
        c.spin_time('ccw', 5.0, 0.35)
    assert c.pulse('ccw', 0.25) is None and c.pulse('cw', 0.35) is None
    c.save()
    d = Calibration.load(tmp_path / 'wood.yaml')
    assert d.pulse('ccw', 0.35) == c.pulse('ccw', 0.35)
    assert d.spin_time('ccw', 90.0, 0.35)[0] == pytest.approx(T)
