"""Put panel/ on sys.path so tests can `import foc_panel`, `import lifecycle`,
and `import sim.*` regardless of pytest's invocation directory."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
