"""find_serial_port(): $FOC_PORT > CH340 VID > name heuristics > first port > fallback."""
import pytest
import foc_panel


class FakePort:
    def __init__(self, device, vid=None):
        self.device = device
        self.vid = vid
        self.pid = None


@pytest.fixture
def no_env(monkeypatch):
    monkeypatch.delenv('FOC_PORT', raising=False)


def _ports(monkeypatch, ports):
    monkeypatch.setattr(foc_panel.list_ports, 'comports', lambda: list(ports))


def test_env_override_wins(monkeypatch):
    monkeypatch.setenv('FOC_PORT', '/dev/ttyMINE')
    _ports(monkeypatch, [FakePort('/dev/ttyUSB0', vid=0x1A86)])
    assert foc_panel.find_serial_port() == '/dev/ttyMINE'


def test_ch340_vid_match(monkeypatch, no_env):
    _ports(monkeypatch, [FakePort('/dev/ttyACM0'), FakePort('/dev/ttyUSB3', vid=0x1A86)])
    assert foc_panel.find_serial_port() == '/dev/ttyUSB3'


def test_name_heuristic_when_no_vid(monkeypatch, no_env):
    _ports(monkeypatch, [FakePort('/dev/cu.Bluetooth'), FakePort('/dev/tty.wchusbserial1420')])
    assert foc_panel.find_serial_port() == '/dev/tty.wchusbserial1420'


def test_first_port_when_no_match(monkeypatch, no_env):
    _ports(monkeypatch, [FakePort('/dev/weird0'), FakePort('/dev/weird1')])
    assert foc_panel.find_serial_port() == '/dev/weird0'


def test_fallback_when_no_ports(monkeypatch, no_env):
    _ports(monkeypatch, [])
    assert foc_panel.find_serial_port() == '/dev/ttyUSB0'


def test_vid_beats_name_heuristic(monkeypatch, no_env):
    # a name-matching port and a VID-matching port -> VID wins (checked first)
    _ports(monkeypatch, [FakePort('/dev/ttyUSB0'), FakePort('/dev/ttyACM9', vid=0x1A86)])
    assert foc_panel.find_serial_port() == '/dev/ttyACM9'
