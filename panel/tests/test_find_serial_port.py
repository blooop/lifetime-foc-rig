"""find_serial_port(): $FOC_PORT > CH340 VID > name heuristics > first port > None.
resolve_backend(): FOC_SIM override > auto (port present = real rig, none = modeled)."""
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


def test_first_usb_port_when_no_match(monkeypatch, no_env):
    # odd name but a real USB VID -> still a plausible board
    _ports(monkeypatch, [FakePort('/dev/weird0'), FakePort('/dev/weird1', vid=0x0403)])
    assert foc_panel.find_serial_port() == '/dev/weird1'


def test_none_when_no_ports(monkeypatch, no_env):
    # no serial hardware at all -> None, the trigger for the modeled rig
    _ports(monkeypatch, [])
    assert foc_panel.find_serial_port() is None


def test_phantom_legacy_ports_dont_count(monkeypatch, no_env):
    # Linux lists 32 motherboard /dev/ttyS* UARTs with no USB VID; they must not
    # be mistaken for the rig (that would silently mask the modeled-rig fallback)
    _ports(monkeypatch, [FakePort(f'/dev/ttyS{i}') for i in range(32)])
    assert foc_panel.find_serial_port() is None


def test_vid_beats_name_heuristic(monkeypatch, no_env):
    # a name-matching port and a VID-matching port -> VID wins (checked first)
    _ports(monkeypatch, [FakePort('/dev/ttyUSB0'), FakePort('/dev/ttyACM9', vid=0x1A86)])
    assert foc_panel.find_serial_port() == '/dev/ttyACM9'


# ---- resolve_backend: the one decision point for real-vs-modeled rig ----

@pytest.fixture
def no_sim_env(monkeypatch):
    monkeypatch.delenv('FOC_SIM', raising=False)


def test_backend_auto_board_present(monkeypatch, no_env, no_sim_env):
    _ports(monkeypatch, [FakePort('/dev/ttyUSB0', vid=0x1A86)])
    assert foc_panel.resolve_backend() == (False, '/dev/ttyUSB0')


def test_backend_auto_no_board(monkeypatch, no_env, no_sim_env):
    _ports(monkeypatch, [])
    assert foc_panel.resolve_backend() == (True, None)


def test_backend_forced_sim_despite_board(monkeypatch, no_env):
    monkeypatch.setenv('FOC_SIM', '1')
    _ports(monkeypatch, [FakePort('/dev/ttyUSB0', vid=0x1A86)])
    assert foc_panel.resolve_backend() == (True, None)


def test_backend_forced_hardware_without_board(monkeypatch, no_env):
    monkeypatch.setenv('FOC_SIM', '0')
    _ports(monkeypatch, [])
    assert foc_panel.resolve_backend() == (False, None)   # worker waits/retries for a board


def test_backend_port_override_means_hardware(monkeypatch, no_sim_env):
    _ports(monkeypatch, [])
    assert foc_panel.resolve_backend('/dev/ttyMINE') == (False, '/dev/ttyMINE')
