"""ziegler_nichols(): relay-feedback -> ZN PI gains."""
import math
import pytest
from foc_panel import ziegler_nichols


def test_none_when_too_few_crossings():
    # need >= warmup + 2 up-crossings
    assert ziegler_nichols([0.0, 1.0, 2.0], vmin=-5, vmax=5, d=0.4, eps=0.4, warmup=2) is None


def test_clean_oscillation_numbers():
    ups = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]   # period = 1.0 s
    r = ziegler_nichols(ups, vmin=-5.0, vmax=5.0, d=0.4, eps=0.4, warmup=2)
    assert r is not None
    a = 5.0
    denom = math.sqrt(a * a - 0.4 * 0.4)
    Ku = 4.0 * 0.4 / (math.pi * denom)
    assert r['a'] == pytest.approx(a)
    assert r['Tu'] == pytest.approx(1.0)
    assert r['Ku'] == pytest.approx(Ku)
    assert r['P'] == pytest.approx(0.45 * Ku)
    assert r['I'] == pytest.approx(r['P'] / (2.2 * r['Tu']))
    assert r['cycles'] == 3            # periods[2:] of 5 periods


def test_warmup_discards_leading_periods():
    # leading periods of 10 s, steady periods of 1 s; warmup=2 must drop the 10s ones
    ups = [0.0, 10.0, 20.0, 21.0, 22.0, 23.0]
    r = ziegler_nichols(ups, vmin=-3, vmax=3, d=0.4, eps=0.4, warmup=2)
    assert r['Tu'] == pytest.approx(1.0)
    assert r['cycles'] == 3


def test_eps_at_or_above_amplitude_uses_floor_no_crash():
    # eps^2 >= a^2 -> denom floored to sqrt(1e-6); must stay finite, not raise
    r = ziegler_nichols([0, 1, 2, 3, 4, 5], vmin=-0.1, vmax=0.1, d=0.4, eps=0.4, warmup=2)
    assert math.isfinite(r['Ku']) and r['Ku'] > 0


def test_zero_period_guards_division():
    # collapsed/repeated up-crossings -> Tu == 0; the `I = P/(2.2*Tu) if Tu>0`
    # guard must fall back to I == 0 (not divide by zero) and stay finite.
    r = ziegler_nichols([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vmin=-2, vmax=2, d=0.4, eps=0.2, warmup=2)
    assert r is not None
    assert r['Tu'] == 0.0
    assert r['I'] == 0.0
    assert math.isfinite(r['P']) and math.isfinite(r['Ku'])


def test_pi_only_no_derivative_term_implied():
    # ZN PI: P = 0.45 Ku, I = P / (2.2 Tu) — assert the canonical ratios hold
    r = ziegler_nichols([0, 0.5, 1.0, 1.5, 2.0, 2.5], vmin=-2, vmax=2, d=0.3, eps=0.2, warmup=1)
    assert r['P'] / r['Ku'] == pytest.approx(0.45)
    assert r['I'] * (2.2 * r['Tu']) == pytest.approx(r['P'])
