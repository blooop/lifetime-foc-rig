"""plot_lifecycle: run-dir resolution + CSV->window replay (no GUI shown)."""
import os
import math
import json
import pytest
import plot_lifecycle as pl


SUMMARY = (
    "cycle,t_s,min_angle,max_angle,span,E_fwd,E_back,peak_tau,mean_tau\n"
    "1,0.50,0.00000,100.00000,100.00000,1.5,1.4,0.5,0.3\n"
    "2,1.00,0.10000,100.20000,100.10000,1.6,1.5,0.6,0.35\n"
    "3,1.50,0.20000,100.40000,,1.7,,0.7,0.4\n"          # missing span + E_back
)
PROFILE = (
    "cycle,dir,bin,pos,tau\n"
    "1,to_max,0,-50.0,0.1\n"
    "1,to_max,1,-49.0,0.2\n"
    "1,to_min,0,-50.0,-0.1\n"
    "2,to_max,0,-50.0,0.15\n"
)


def make_run(root, name='20260101_120000', config=True):
    d = os.path.join(root, name)
    os.makedirs(d)
    with open(os.path.join(d, 'summary.csv'), 'w') as f:
        f.write(SUMMARY)
    with open(os.path.join(d, 'profile.csv'), 'w') as f:
        f.write(PROFILE)
    if config:
        with open(os.path.join(d, 'config.json'), 'w') as f:
            json.dump({'n_bins': 4}, f)
    return d


class FakeWin:
    def __init__(self):
        self.statuses = []
        self.profiles = []

    def add_status(self, st):
        self.statuses.append(st)

    def add_profile(self, cycle, row):
        self.profiles.append((cycle, row))


# ------------------------------------------------------------------- _fnum

@pytest.mark.parametrize('s,expected', [('1.5', 1.5), ('0', 0.0), ('-2.25', -2.25)])
def test_fnum_parses(s, expected):
    assert pl._fnum(s) == expected


@pytest.mark.parametrize('s', ['', 'x', None])
def test_fnum_bad_is_nan(s):
    assert math.isnan(pl._fnum(s))


# ------------------------------------------------------------------- resolution

def test_latest_run_picks_newest(tmp_path, monkeypatch):
    root = str(tmp_path)
    monkeypatch.setattr(pl, 'RUNS_ROOT', root)
    old = make_run(root, 'old')
    new = make_run(root, 'new')
    os.utime(old, (1000, 1000))
    os.utime(new, (2000, 2000))
    assert pl._latest_run() == new


def test_latest_run_none_when_empty(tmp_path, monkeypatch):
    monkeypatch.setattr(pl, 'RUNS_ROOT', str(tmp_path))
    assert pl._latest_run() is None


def test_resolve_absolute(tmp_path, monkeypatch):
    root = str(tmp_path)
    monkeypatch.setattr(pl, 'RUNS_ROOT', root)
    d = make_run(root, 'run_abs')
    assert pl._resolve(d) == d


def test_resolve_basename_relative_to_root(tmp_path, monkeypatch):
    root = str(tmp_path)
    monkeypatch.setattr(pl, 'RUNS_ROOT', root)
    d = make_run(root, 'run_rel')
    assert pl._resolve('run_rel') == d
    assert pl._resolve('/some/other/run_rel') == d   # falls back to basename under root


def test_resolve_empty_uses_latest(tmp_path, monkeypatch):
    root = str(tmp_path)
    monkeypatch.setattr(pl, 'RUNS_ROOT', root)
    d = make_run(root, 'only')
    assert pl._resolve('') == d


# ------------------------------------------------------------------- load()

def test_load_replays_summary_and_profile(tmp_path):
    d = make_run(str(tmp_path))
    win = FakeWin()
    pl.load(win, d)
    # one add_status per summary row
    assert [s['cycle'] for s in win.statuses] == [1, 2, 3]
    assert all(s['phase'] == 'run' for s in win.statuses)
    # missing span comes through as NaN (the real window filters it; load doesn't)
    assert math.isnan(win.statuses[2]['span'])
    assert math.isnan(win.statuses[2]['E_back'])
    # profile: one row per cycle present, each row width == n_bins from config.json
    assert [c for c, _ in win.profiles] == [1, 2]
    assert all(len(row) == 4 for _, row in win.profiles)
    # bins with no samples are NaN; bin 0 of cycle 1 has data
    c1 = dict(win.profiles)[1]
    assert not math.isnan(c1[0])
    assert math.isnan(c1[3])


def test_load_defaults_nbins_without_config(tmp_path):
    d = make_run(str(tmp_path), name='noconfig', config=False)
    win = FakeWin()
    pl.load(win, d)
    assert all(len(row) == 100 for _, row in win.profiles)   # default n_bins
