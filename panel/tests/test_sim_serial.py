"""SimSerial speaks the board's serial protocol through its threaded clock, the
same way SerialWorker drives it. Uses the AnalyticPlant at high speed so it's fast
and dependency-free.
"""
import time

from sim.sim_serial import SimSerial, configure


def _collect(s, seconds, writes=()):
    for w in writes:
        s.write(w)
    seen = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = s.readline()
        if raw:
            seen.append(raw.decode(errors="replace").strip())
    return seen


def _open(speed=50):
    configure(plant="analytic", speed=speed, control_hz=1000, on_step=None, viewer=False)
    s = SimSerial()
    s.port, s.baudrate, s.timeout = "sim", 115200, 0.05   # as SerialWorker sets them
    s.dtr = False
    s.rts = False
    s.open()
    return s


def test_simserial_emits_protocol():
    s = _open()
    try:
        seen = _collect(s, 1.5, writes=[b"ME1\n", b"MC1\n", b"M10\n"])
    finally:
        s.close()
    assert any("Motor ready" in l for l in seen), "no boot/Motor ready line"
    telem = [l for l in seen if "\t" in l and not l.startswith(("E\t", "S\t"))]
    assert telem, "no 7-var monitor line"
    assert len(telem[-1].split("\t")) == 7
    assert any(l.startswith("E\t") for l in seen), "no E (endstop) line"


def test_simserial_rts_pulse_reboots():
    s = _open()
    try:
        _collect(s, 0.3)                      # let it boot
        s.rts = True                          # reset pulse (rising edge)
        s.rts = False
        seen = _collect(s, 0.8)
    finally:
        s.close()
    assert any("Motor ready" in l for l in seen), "RTS pulse did not reboot the board"
