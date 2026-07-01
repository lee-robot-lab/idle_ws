"""watch-style monitor rendering tests."""

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib.parse import Feedback
from monitor import WatchRow, format_watch_screen


class MonitorWatchTest(unittest.TestCase):
    def test_format_watch_screen_keeps_motor_rows_stable(self):
        rows = {
            2: WatchRow(
                feedback=Feedback(
                    motor_id=2,
                    host_id=0xFD,
                    mode_status=1,
                    fault_bits=0,
                    pos=0.1234,
                    vel=-0.2,
                    tor=1.5,
                    temp_c=32.0,
                ),
                last_seen_s=9.8,
                source_type=0x02,
            ),
            1: WatchRow(
                feedback=Feedback(
                    motor_id=1,
                    host_id=0xFD,
                    mode_status=2,
                    fault_bits=3,
                    pos=-0.5,
                    vel=0.0,
                    tor=-0.25,
                    temp_c=30.5,
                ),
                last_seen_s=9.0,
                source_type=0x18,
            ),
        }

        text = format_watch_screen(rows, now_s=10.0, channel="can0", can_ids=[1, 2], stale_s=0.5)

        self.assertIn("ID", text)
        self.assertLess(text.index("\n 1"), text.index("\n 2"))
        self.assertIn("0x03", text)
        self.assertIn("STALE", text)
        self.assertIn("+0.1234", text)


if __name__ == "__main__":
    unittest.main()
