"""kt_from_kv() + model_iq(): the shared Vq torque-model helpers."""
import pytest
from lifecycle import kt_from_kv, model_iq, LifecycleController, LifecycleConfig


def test_kt_from_kv_nameplate():
    assert kt_from_kv(1000.0) == pytest.approx(9.549 / 1000.0)


def test_kt_override_wins_when_positive():
    assert kt_from_kv(1000.0, override=0.012) == pytest.approx(0.012)


def test_kt_override_zero_uses_kv():
    assert kt_from_kv(500.0, override=0.0) == pytest.approx(9.549 / 500.0)


def test_kt_kv_floored_to_one():
    # KV clamped to >= 1 so we never divide by zero
    assert kt_from_kv(0.0) == pytest.approx(9.549)


def test_model_iq_basic():
    # Iq = (Vq - Ke*w) / R
    assert model_iq(vq=2.0, velocity=10.0, kt=0.01, r=0.15) == pytest.approx((2.0 - 0.1) / 0.15)


def test_model_iq_zero_velocity():
    assert model_iq(vq=1.5, velocity=0.0, kt=0.0095, r=0.2) == pytest.approx(1.5 / 0.2)


def test_model_iq_r_floored():
    # R floored to 1e-3 -> no divide-by-zero blowup, large but finite
    val = model_iq(vq=1.0, velocity=0.0, kt=0.01, r=0.0)
    assert val == pytest.approx(1.0 / 1e-3)


def test_controller_kt_and_r_providers():
    cfg = LifecycleConfig(kv=1000.0, kt_override=0.0, phase_resistance=0.15)
    lc = LifecycleController(cfg)
    assert lc._kt() == pytest.approx(9.549 / 1000.0)
    assert lc._r() == pytest.approx(0.15)
    # live providers override the config
    lc.kt_provider = lambda: 0.02
    lc.r_provider = lambda: 0.3
    assert lc._kt() == 0.02
    assert lc._r() == 0.3
    # r provider floored
    lc.r_provider = lambda: 0.0
    assert lc._r() == pytest.approx(1e-3)
