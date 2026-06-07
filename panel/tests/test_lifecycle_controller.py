"""LifecycleController state machine, metrics, abort paths, persistence.

Driven entirely through a FakeWorker (emit signals / record sends) + a controllable
clock — no serial, no board, no Qt event loop. Signals fire synchronously on emit().
"""
import os
import csv
import json
import math
import pytest
from lifecycle import LifecycleController, LifecycleConfig


def make_lc(tmp_run_dir, **kw):
    return LifecycleController(LifecycleConfig(out_root=tmp_run_dir, **kw))


def reasons_of(lc):
    out = []
    lc.finished.connect(out.append)
    return out


def home(lc, worker):
    """Drive a started controller through homing into the run phase."""
    worker.endstop.emit(0, 0, 0, 1, 5.0, 0)    # seeking MIN (phase 1) -> _home_started
    worker.endstop.emit(0, 0, 1, 0, 0.0, 0)    # homed at idle -> _begin_run
    assert lc.phase == 'run'


def samples(worker, vq=0.5, v=2.0, n=3, a0=0.0, da=1.0):
    """Feed a few telemetry points so the next finalized stroke has energy."""
    a = a0
    for _ in range(n):
        worker.telem.emit(0.0, vq, 0.0, v, a)   # target, Vq, Iq_meas, velocity, angle
        a += da


# --------------------------------------------------------------------------- homing

def test_homing_sends_expected_commands(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0)
    lc.start(worker)
    assert lc.phase == 'homing'
    for c in ('PE1', 'MC1', 'ME1', 'EH'):
        assert c in worker.sent
    assert any(c.startswith('MLV') for c in worker.sent)


def test_homing_to_run_transition(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0)
    lc.start(worker)
    worker.endstop.emit(0, 0, 0, 1, 5.0, 0)
    assert lc._home_started and lc.phase == 'homing'
    worker.endstop.emit(0, 0, 1, 0, 0.0, 0)
    assert lc.phase == 'run'
    assert lc.direction == 1
    assert worker.last() == 'M3.000'           # M{direction * v_measure}


# --------------------------------------------------------------------------- cycles

def test_cycle_counting_and_direction_flips(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=5)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0)        # first contact defines the boundary (MIN)
    assert lc.start_end == 0 and lc.direction == -1
    worker.slip.emit(1, 100.0)      # MAX
    assert lc.direction == 1 and lc.cycle == 0
    worker.slip.emit(0, 0.0)        # back to MIN -> one full cycle
    assert lc.cycle == 1 and lc.direction == -1
    assert lc.running


def test_baseline_span_and_energy(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=5)
    statuses = []
    lc.status.connect(statuses.append)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0)        # start boundary
    samples(worker)                 # forward stroke energy
    worker.slip.emit(1, 100.0)      # finalize to_max
    samples(worker)                 # back stroke energy
    worker.slip.emit(0, 0.0)        # finalize to_min -> complete cycle 1
    assert lc.cycle == 1
    assert lc.span0 == pytest.approx(100.0)
    last = statuses[-1]
    assert last['cycle'] == 1
    assert math.isfinite(last['E_fwd']) and last['E_fwd'] != 0.0
    assert math.isfinite(last['E_back'])


def test_clean_completion_parks_then_finishes(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=1)
    reasons = reasons_of(lc)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0)
    worker.slip.emit(1, 100.0)
    worker.slip.emit(0, 0.0)        # cycle 1 == target -> park
    assert lc.phase == 'parking'
    assert 'MC2' in worker.sent
    assert any(c.startswith('M50') for c in worker.sent)   # center = (0+100)/2
    assert lc.running                                      # not done until centered
    worker.endstop.emit(0, 0, 1, 0, 0.0, 0)               # arrived at center (pos~0)
    assert not lc.running
    assert reasons == ['target reached (1 cycles)']
    # a real STOP disables the driver, not just M0
    for c in ('EX', 'M0', 'ME0'):
        assert c in worker.sent


# --------------------------------------------------------------------------- aborts

def test_sustained_iq_anomaly_aborts(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=5.0, phase_resistance=0.15, kv=1000.0)
    reasons = reasons_of(lc)
    lc.start(worker); home(lc, worker)
    # iq_model = (Vq - Ke*w)/R = 2.0/0.15 ~= 13 A > 5 A
    worker.telem.emit(0.0, 2.0, 0.0, 0.0, 0.0)
    assert lc.running                       # single sample over threshold: not yet
    clock.advance(0.6)                      # > IQ_DWELL_S (0.5)
    worker.telem.emit(0.0, 2.0, 0.0, 0.0, 1.0)
    assert not lc.running
    assert 'anomaly' in reasons[0]
    assert 'ME0' in worker.sent


def test_brief_iq_spike_does_not_abort(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=5.0, phase_resistance=0.15, kv=1000.0)
    lc.start(worker); home(lc, worker)
    worker.telem.emit(0.0, 2.0, 0.0, 0.0, 0.0)   # over threshold
    clock.advance(0.2)
    worker.telem.emit(0.0, 0.0, 0.0, 0.0, 1.0)   # back under -> resets the dwell timer
    clock.advance(1.0)
    worker.telem.emit(0.0, 2.0, 0.0, 0.0, 2.0)   # over again, but dwell just restarted
    assert lc.running


def test_slip_span_anomaly_aborts(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=10,
                 slip_abort_frac=0.20)
    reasons = reasons_of(lc)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0)
    worker.slip.emit(1, 100.0)
    worker.slip.emit(0, 0.0)        # cycle 1: span0 = 100
    assert lc.span0 == pytest.approx(100.0)
    worker.slip.emit(1, 130.0)      # cycle 2: span jumps to 130 (> 20% drift)
    worker.slip.emit(0, 0.0)
    assert not lc.running
    assert 'slip/span anomaly' in reasons[0]


def test_backstop_fired_aborts(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0)
    reasons = reasons_of(lc)
    lc.start(worker); home(lc, worker)
    worker.endstop.emit(0, 0, 1, 0, 5.0, 1)     # backstop=1 -> past MIN
    assert not lc.running
    assert 'overtravel backstop' in reasons[0] and 'MIN' in reasons[0]


def test_stall_detector_aborts_via_heartbeat(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, stall_timeout_s=6.0, prog_eps=0.5)
    reasons = reasons_of(lc)
    lc.start(worker); home(lc, worker)
    worker.endstop.emit(0, 0, 1, 0, 10.0, 0)    # records a progress point
    clock.advance(7.0)                          # no further progress for > stall_timeout
    lc._heartbeat()
    assert not lc.running
    assert 'stalled' in reasons[0]


def test_heartbeat_reissues_when_progressing(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, stall_timeout_s=6.0)
    lc.start(worker); home(lc, worker)
    worker.endstop.emit(0, 0, 1, 0, 10.0, 0)
    clock.advance(2.0)
    worker.sent_clear()
    lc._heartbeat()
    assert lc.running and worker.last().startswith('M')


# --------------------------------------------------------------------------- persistence

def test_csv_and_state_written(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=5)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0); worker.slip.emit(1, 100.0); worker.slip.emit(0, 0.0)
    run_dir = lc.run_dir
    with open(os.path.join(run_dir, 'summary.csv')) as f:
        rows = list(csv.reader(f))
    assert rows[0] == ['cycle', 't_s', 'min_angle', 'max_angle', 'span',
                       'E_fwd', 'E_back', 'peak_tau', 'mean_tau']
    assert rows[1][0] == '1'
    assert os.path.exists(os.path.join(run_dir, 'profile.csv'))
    with open(os.path.join(run_dir, 'state.json')) as f:
        assert json.load(f)['cycle'] == 1
    assert os.path.exists(os.path.join(run_dir, 'config.json'))


def test_resume_restores_cycle(tmp_run_dir, worker, clock):
    prev = os.path.join(tmp_run_dir, 'prev')
    os.makedirs(prev)
    with open(os.path.join(prev, 'state.json'), 'w') as f:
        json.dump({'cycle': 42, 'span0': 100.0, 'E0': 5.0}, f)
    lc = make_lc(tmp_run_dir, resume_dir=prev)
    lc.start(worker)
    assert lc.cycle == 42
    assert lc.span0 == pytest.approx(100.0)
    assert lc.run_dir == prev


def test_rehome_on_board_reset_keeps_cycle_count(tmp_run_dir, worker, clock):
    lc = make_lc(tmp_run_dir, v_measure=3.0, iq_abort=1e9, target_cycles=10)
    lc.start(worker); home(lc, worker)
    worker.slip.emit(0, 0.0); worker.slip.emit(1, 100.0); worker.slip.emit(0, 0.0)
    assert lc.cycle == 1
    worker.sent_clear()
    worker.ready.emit()                 # board reset detected mid-run
    assert lc.phase == 'homing'
    assert 'EH' in worker.sent
    assert lc.cycle == 1                 # counter survives the reset
