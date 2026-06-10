"""RigView: geometry is learned from E-line telemetry (hall markers snap to the
reported position on a trigger while homed), and the SIM badge reflects the
worker's real-vs-modeled decision. Offscreen Qt, no board."""
import pytest

from rig_view import RigView, MIN, MAX, OVERTRAVEL_FRAC, SEED_HALL


@pytest.fixture
def view(qtbot):
    v = RigView()
    qtbot.addWidget(v)
    return v


def test_seeds_until_learned(view):
    assert view.hall == list(SEED_HALL)
    assert view._learned == [False, False]


def test_learns_hall_positions_when_homed(view):
    # cycling: MIN trigger at +98.5, later MAX trigger at -101.0
    view.on_endstop(1, 0, 1, 0, 98.5, 0)
    view.on_endstop(0, 1, 1, 0, -101.0, 0)
    assert view.hall == [98.5, -101.0]
    assert view._learned == [True, True]
    # markers + backstops follow the learned geometry
    travel = 98.5 - (-101.0)
    assert view.hall_lines[MIN].value() == pytest.approx(98.5)
    assert view.hall_lines[MAX].value() == pytest.approx(-101.0)
    assert view.stop_lines[MIN].value() == pytest.approx(98.5 + OVERTRAVEL_FRAC * travel)
    assert view.stop_lines[MAX].value() == pytest.approx(-101.0 - OVERTRAVEL_FRAC * travel)


def test_no_learning_before_home_or_during_homing(view):
    view.on_endstop(1, 0, 0, 0, 250.0, 0)   # not homed (frame is boot-relative)
    view.on_endstop(1, 0, 1, 1, 250.0, 0)   # homing redefines the zero mid-sweep
    assert view.hall == list(SEED_HALL)
    assert view._learned == [False, False]


def test_relearning_tracks_slip(view):
    view.on_endstop(1, 0, 1, 0, 98.5, 0)
    view.on_endstop(1, 0, 1, 0, 99.7, 0)    # still in the zone -> NOT relearned
    assert view.hall[MIN] == pytest.approx(98.5)   # rising-edge latch only
    view.on_endstop(0, 0, 1, 0, 50.0, 0)    # back away (clear)
    view.on_endstop(1, 0, 1, 0, 97.9, 0)    # next stroke: hall drifted -> marker follows
    assert view.hall[MIN] == pytest.approx(97.9)


def test_sim_badge(view):
    view.set_sim(True)
    assert 'SIM' in view.badge.text()
    view.set_sim(False)
    assert 'REAL RIG' in view.badge.text()
