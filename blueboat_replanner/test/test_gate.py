"""The offline gate as one test, so that `colcon test` runs it (pytest or unittest).

The gate runs in a subprocess: section [24] installs fake rclpy modules, which must not leak into
the test process. The gate's failing lines are shown on failure.
"""

import os
import subprocess
import sys
import unittest
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]


class OfflineGate(unittest.TestCase):
    def test_offline_gate(self):
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        r = subprocess.run([sys.executable, "-B", "-m", "test.smoke_test"], cwd=PKG_ROOT,
                           env=env, capture_output=True, text=True, timeout=1200)
        out = r.stdout + r.stderr
        tail = "\n".join(line for line in out.splitlines()
                         if "FAIL" in line or "SMOKE" in line or "Error" in line)[-6000:]
        self.assertEqual(r.returncode, 0, tail)
        self.assertIn(" 0 failed", out.splitlines()[-1] if out.strip() else "", tail)


if __name__ == "__main__":
    unittest.main()
