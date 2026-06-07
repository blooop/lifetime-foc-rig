"""Shared fixtures for the hardware-free panel test suite.

Everything here lets the PyQt-coupled code run with no board and no display:
  - a process-wide QApplication (so QObject signals/slots work),
  - a FakeWorker that mimics SerialWorker's signals + records every send(), and
  - a controllable monotonic clock so time-based logic (dwell/stall/heartbeat)
    is deterministic.

Signals connected within one thread fire synchronously on .emit(), so the tests
drive the controllers by emitting on the FakeWorker (or calling slots directly) —
no Qt event loop required.
"""
import os
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtWidgets


@pytest.fixture(scope='session')
def qapp():
    """One QApplication for the whole session (needed for QWidget-based tests)."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


@pytest.fixture(autouse=True, scope='session')
def _qcore():
    """Guarantee a QCoreApplication exists for the signal-only controller tests."""
    app = QtCore.QCoreApplication.instance() or QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
    yield app


class FakeWorker(QtCore.QObject):
    """Stand-in for foc_panel.SerialWorker.

    Same signal surface the LifecycleController/Panel connect to, plus a send()
    that records commands into .sent for assertions. No serial, no thread.
    """
    line = QtCore.pyqtSignal(str)
    telem = QtCore.pyqtSignal(float, float, float, float, float)
    endstop = QtCore.pyqtSignal(int, int, int, int, float, int)
    slip = QtCore.pyqtSignal(int, float)
    ready = QtCore.pyqtSignal()
    tune_status = QtCore.pyqtSignal(str)
    tune_done = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.sent = []

    def send(self, cmd):
        self.sent.append(cmd)

    # convenience for assertions
    def sent_clear(self):
        self.sent.clear()

    def last(self):
        return self.sent[-1] if self.sent else None


@pytest.fixture
def worker():
    return FakeWorker()


class FakeClock:
    """Monotonic clock the tests advance by hand."""
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt
        return self.t


@pytest.fixture
def clock(monkeypatch):
    """Patch time.monotonic in lifecycle (and foc_panel) to a controllable clock."""
    import lifecycle
    clk = FakeClock()
    monkeypatch.setattr(lifecycle.time, 'monotonic', clk)
    try:
        import foc_panel
        monkeypatch.setattr(foc_panel.time, 'monotonic', clk)
    except Exception:
        pass
    return clk


@pytest.fixture(autouse=True)
def _no_sleep_inhibit(monkeypatch):
    """Never spawn a real systemd-inhibit/caffeinate subprocess during tests."""
    import lifecycle
    monkeypatch.setattr(lifecycle.LifecycleController, '_inhibit_sleep', lambda self: None)


@pytest.fixture
def tmp_run_dir(tmp_path):
    """A throwaway lifecycle_runs root for CSV/state output."""
    d = tmp_path / 'lifecycle_runs'
    d.mkdir()
    return str(d)
