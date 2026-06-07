"""parse_line(): the firmware serial-line classifier (the mA->A conversion bug guard)."""
import pytest
from foc_panel import parse_line


def test_endstop_full_with_backstop():
    kind, payload = parse_line('E\t1\t0\t1\t3\t12.500\t2')
    assert kind == 'endstop'
    assert payload == (1, 0, 1, 3, 12.5, 2)


def test_endstop_without_backstop_field_defaults_zero():
    # firmware < v3 omits the trailing backstop field
    kind, payload = parse_line('E\t0\t1\t0\t0\t-4.25')
    assert kind == 'endstop'
    assert payload == (0, 1, 0, 0, -4.25, 0)


def test_endstop_too_few_fields_falls_back_to_line():
    kind, payload = parse_line('E\t1\t0')
    assert kind == 'line'
    assert payload == 'E\t1\t0'


def test_endstop_malformed_value_falls_back_to_line():
    kind, payload = parse_line('E\t1\tx\t1\t0\t1.0')
    assert kind == 'line'


def test_slip_line():
    kind, payload = parse_line('S\t1\t37.125')
    assert kind == 'slip'
    assert payload == (1, 37.125)


def test_slip_malformed_falls_back():
    assert parse_line('S\t1')[0] == 'line'
    assert parse_line('S\tmin\t1.0')[0] == 'line'


def test_telem_converts_iq_milliamps_to_amps():
    # 7-var monitor: target, Vq, Id, Iq[mA], (extra), velocity, angle
    # index 3 is Iq in mA -> must come back as amps; velocity=v[5], angle=v[6].
    line = '\t'.join(['2.0', '1.6', '0.0', '3210.0', '0.0', '54.2', '12.34'])
    kind, payload = parse_line(line)
    assert kind == 'telem'
    target, vq, iq_a, vel, ang = payload
    assert target == 2.0
    assert vq == 1.6
    assert iq_a == pytest.approx(3.21)      # 3210 mA -> 3.21 A
    assert vel == 54.2
    assert ang == 12.34


def test_telem_non_numeric_falls_back_to_line():
    kind, payload = parse_line('\t'.join(['a'] * 7))
    assert kind == 'line'


def test_short_line_is_plain():
    assert parse_line('Motor ready')[0] == 'line'
    assert parse_line('sensor_direction==CCW')[0] == 'line'


def test_six_field_non_e_line_is_plain():
    # 6 tab-fields and not an E/S line -> not a 7-var monitor, so a plain line
    assert parse_line('\t'.join(['1.0'] * 6))[0] == 'line'
