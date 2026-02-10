from pathlib import Path
import sys
import os
import json
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib.control_tuning import (
    assert_control_ready_for_command,
    control_params_dirty,
    save_control_tuning,
    set_control_tuning,
)
from lib.param_store import control_tuned_path, load_control_tuned


class ControlTuningWorkflowTest(unittest.TestCase):
    def test_modify_save_then_control(self):
        old_param_root = os.environ.get("IDLE_PARAM_ROOT")
        old_state_path = os.environ.get("IDLE_CONTROL_GATE_STATE")
        try:
            with tempfile.TemporaryDirectory() as td:
                param_root = Path(td) / "param"
                state_file = Path(td) / "control_state.json"
                os.environ["IDLE_PARAM_ROOT"] = str(param_root)
                os.environ["IDLE_CONTROL_GATE_STATE"] = str(state_file)

                set_control_tuning([1], {"kp": 12.5, "kd": 0.2})
                self.assertTrue(control_params_dirty())

                with self.assertRaises(RuntimeError):
                    assert_control_ready_for_command()

                save_control_tuning()
                self.assertFalse(control_params_dirty())
                assert_control_ready_for_command()

                tuned = load_control_tuned()
                self.assertIn("motors", tuned)
                self.assertAlmostEqual(float(tuned["motors"]["1"]["kp"]), 12.5)
                self.assertAlmostEqual(float(tuned["motors"]["1"]["kd"]), 0.2)
                self.assertTrue(control_tuned_path().exists())

                with state_file.open("r", encoding="utf-8") as fp:
                    state = json.load(fp)
                self.assertIn("tx_hz_default", state)
                self.assertGreater(float(state["tx_hz_default"]), 0.0)
                self.assertIn("tx_hz_by_motor", state)
                self.assertIsInstance(state["tx_hz_by_motor"], dict)
        finally:
            if old_param_root is None:
                os.environ.pop("IDLE_PARAM_ROOT", None)
            else:
                os.environ["IDLE_PARAM_ROOT"] = old_param_root
            if old_state_path is None:
                os.environ.pop("IDLE_CONTROL_GATE_STATE", None)
            else:
                os.environ["IDLE_CONTROL_GATE_STATE"] = old_state_path

    def test_save_updates_tx_policy_for_bridge(self):
        old_param_root = os.environ.get("IDLE_PARAM_ROOT")
        old_state_path = os.environ.get("IDLE_CONTROL_GATE_STATE")
        try:
            with tempfile.TemporaryDirectory() as td:
                param_root = Path(td) / "param"
                state_file = Path(td) / "control_state.json"
                os.environ["IDLE_PARAM_ROOT"] = str(param_root)
                os.environ["IDLE_CONTROL_GATE_STATE"] = str(state_file)

                set_control_tuning([3], {"tx_hz": 180.0})
                save_control_tuning()

                with state_file.open("r", encoding="utf-8") as fp:
                    state = json.load(fp)
                self.assertAlmostEqual(float(state["tx_hz_default"]), 500.0)
                self.assertIn("3", state["tx_hz_by_motor"])
                self.assertAlmostEqual(float(state["tx_hz_by_motor"]["3"]), 180.0)
        finally:
            if old_param_root is None:
                os.environ.pop("IDLE_PARAM_ROOT", None)
            else:
                os.environ["IDLE_PARAM_ROOT"] = old_param_root
            if old_state_path is None:
                os.environ.pop("IDLE_CONTROL_GATE_STATE", None)
            else:
                os.environ["IDLE_CONTROL_GATE_STATE"] = old_state_path


if __name__ == "__main__":
    unittest.main()
