"""Run board-independent regressions without configuring hardware."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'src/sgm_yolo'
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / 'tests'))


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.discover(str(PROJECT / 'tests'), pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
