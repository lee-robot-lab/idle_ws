# Adaptive Gain Scheduling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add opt-in CRBA diagonal effective-inertia gain scheduling for `plan_node` while preserving fixed-gain behavior by default.

**Architecture:** Put gain math and LPF state in a small pure-Python module, expose only deterministic functions/classes, then call it from `plan_node` where `kp/kd` are currently loaded. `RobotModel.mass_matrix()` remains the only Pinocchio dependency; `plan_node` computes one mass matrix per tick and reuses the diagonal for all motors.

**Tech Stack:** Python 3, ROS2 `rclpy`, Pinocchio via existing `RobotModel`, pytest, existing YAML control tuning utilities.

---

### Task 1: Pure Adaptive Gain Module

**Files:**
- Create: `src/phy/phy/adaptive_gain.py`
- Test: `src/phy/test/test_adaptive_gain.py`

- [ ] **Step 1: Write failing tests**

Add `src/phy/test/test_adaptive_gain.py`:

```python
import math

from phy.adaptive_gain import AdaptiveGainState, compute_adaptive_gains, lpf_j_eff


def test_compute_adaptive_gains_preserves_target_second_order_params():
    gains = compute_adaptive_gains(j_eff=0.5, omega_n_target=6.0, zeta_target=1.25)

    assert gains == (18.0, 7.5)


def test_compute_adaptive_gains_returns_none_for_invalid_inputs():
    assert compute_adaptive_gains(0.0, 6.0, 1.0) is None
    assert compute_adaptive_gains(float("nan"), 6.0, 1.0) is None
    assert compute_adaptive_gains(0.5, 0.0, 1.0) is None
    assert compute_adaptive_gains(0.5, 6.0, 0.0) is None


def test_lpf_j_eff_initializes_then_blends_when_alpha_between_zero_and_one():
    state = AdaptiveGainState()

    assert lpf_j_eff(state, motor_id=2, raw_j_eff=2.0, alpha=0.25) == 2.0
    assert lpf_j_eff(state, motor_id=2, raw_j_eff=6.0, alpha=0.25) == 3.0


def test_lpf_j_eff_alpha_missing_or_one_uses_raw_value():
    state = AdaptiveGainState()

    assert lpf_j_eff(state, motor_id=1, raw_j_eff=2.0, alpha=None) == 2.0
    assert lpf_j_eff(state, motor_id=1, raw_j_eff=8.0, alpha=1.0) == 8.0


def test_lpf_j_eff_ignores_invalid_raw_and_keeps_previous_value():
    state = AdaptiveGainState(j_eff_lpf={3: 4.0})

    assert lpf_j_eff(state, motor_id=3, raw_j_eff=math.inf, alpha=0.5) == 4.0
```

- [ ] **Step 2: Run test to verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_adaptive_gain.py -q
```

Expected: import failure because `phy.adaptive_gain` does not exist.

- [ ] **Step 3: Implement module**

Create `src/phy/phy/adaptive_gain.py`:

```python
"""Pure adaptive gain scheduling helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class AdaptiveGainState:
    """Runtime LPF state keyed by motor id."""

    j_eff_lpf: dict[int, float] = field(default_factory=dict)


def compute_adaptive_gains(
    j_eff: float,
    omega_n_target: float,
    zeta_target: float,
) -> tuple[float, float] | None:
    values = (j_eff, omega_n_target, zeta_target)
    if not all(math.isfinite(float(v)) and float(v) > 0.0 for v in values):
        return None
    j = float(j_eff)
    omega = float(omega_n_target)
    zeta = float(zeta_target)
    return j * omega * omega, 2.0 * zeta * j * omega


def lpf_j_eff(
    state: AdaptiveGainState,
    *,
    motor_id: int,
    raw_j_eff: float,
    alpha: float | None,
) -> float | None:
    raw = float(raw_j_eff)
    previous = state.j_eff_lpf.get(int(motor_id))
    if not math.isfinite(raw) or raw <= 0.0:
        return previous
    if alpha is None:
        state.j_eff_lpf[int(motor_id)] = raw
        return raw
    a = max(0.0, min(1.0, float(alpha)))
    if previous is None or a >= 1.0:
        value = raw
    else:
        value = previous + a * (raw - previous)
    state.j_eff_lpf[int(motor_id)] = value
    return value
```

- [ ] **Step 4: Run test to verify GREEN**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_adaptive_gain.py -q
```

Expected: all tests pass.

### Task 2: Allow Adaptive Tuning Keys

**Files:**
- Modify: `src/idle_common/idle_common/control_tuning.py`
- Test: `motor/tests/test_control_tuning_workflow.py`

- [ ] **Step 1: Write failing test**

Add to `motor/tests/test_control_tuning_workflow.py`:

```python
def test_control_tuning_accepts_adaptive_gain_keys(self):
    with tempfile.TemporaryDirectory() as td:
        with unittest.mock.patch.dict(os.environ, {
            "IDLE_PARAM_ROOT": td,
            "IDLE_CONTROL_GATE_STATE": str(Path(td) / "gate.json"),
        }):
            set_control_tuning([2], {
                "gain_mode": 1,
                "omega_n_target": 6.0,
                "zeta_target": 1.1,
                "gain_lpf_alpha": 0.25,
            })
            values = control_params_for_motor(2)
            self.assertEqual(values["gain_mode"], 1.0)
            self.assertEqual(values["omega_n_target"], 6.0)
            self.assertEqual(values["zeta_target"], 1.1)
            self.assertEqual(values["gain_lpf_alpha"], 0.25)
```

- [ ] **Step 2: Run test to verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest motor/tests/test_control_tuning_workflow.py::ControlTuningWorkflowTest::test_control_tuning_accepts_adaptive_gain_keys -q
```

Expected: `unsupported control keys`.

- [ ] **Step 3: Add keys and validation**

Add these keys to `CONTROL_TUNING_KEYS`:

```python
"gain_mode",
"omega_n_target",
"zeta_target",
"gain_lpf_alpha",
```

Extend `_validate_tuning_value`:

```python
if key == "gain_mode" and f not in {0.0, 1.0}:
    raise ValueError("gain_mode must be 0 or 1")
if key in {"omega_n_target", "zeta_target"} and f <= 0:
    raise ValueError(f"{key} must be > 0")
if key == "gain_lpf_alpha" and not (0.0 <= f <= 1.0):
    raise ValueError("gain_lpf_alpha must be between 0 and 1")
```

- [ ] **Step 4: Run test to verify GREEN**

Run the same pytest command. Expected: pass.

### Task 3: Add `j_eff` Diagnostics

**Files:**
- Modify: `src/phy/phy/plan_node.py`
- Test: `src/phy/test/test_plan_node_gain_scaling.py`

- [ ] **Step 1: Write failing test update**

Update existing `_diag_csv_rows` expected rows to include a `j_eff` value after `kd`; add a dedicated assertion:

```python
def test_diag_csv_rows_include_j_eff_when_command_provides_it():
    rows = _diag_csv_rows(
        now_s=1.0,
        phase="trajectory",
        vt_s=0.5,
        warp=1.0,
        settle_blend=0.0,
        motor_ids=[2],
        state_by_motor={2: MotorSample(q=0.0, qd=0.0, tau_measured=0.0, last_seen_s=1.0)},
        cmd_values={2: {"q_des": 0.0, "qd_des": 0.0, "kp": 12.0, "kd": 4.0, "j_eff": 0.333333333, "tau_ff": 0.0}},
    )
    assert rows[0][13] == "4.000000"
    assert rows[0][14] == "0.333333333"
```

- [ ] **Step 2: Run test to verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan_node_gain_scaling.py::test_diag_csv_rows_include_j_eff_when_command_provides_it -q
```

Expected: row index/column failure because `j_eff` is not emitted.

- [ ] **Step 3: Add CSV header/row field**

In `_diag_csv_header()`, add `"j_eff"` immediately after `"kd"`.

In `_diag_csv_rows()`, read:

```python
j_eff = cmd.get("j_eff", math.nan)
```

and emit:

```python
"" if not math.isfinite(float(j_eff)) else f"{float(j_eff):.9f}",
```

immediately after `kd`.

- [ ] **Step 4: Run tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan_node_gain_scaling.py -q
```

Expected: pass.

### Task 4: Integrate Adaptive Gains in `plan_node`

**Files:**
- Modify: `src/phy/phy/plan_node.py`
- Test: `src/phy/test/test_plan_node_gain_scaling.py`

- [ ] **Step 1: Write failing pure helper tests**

Add helper-level tests after importing new helper functions:

```python
from phy.plan_node import _adaptive_gain_for_motor


def test_adaptive_gain_for_motor_returns_fixed_gain_when_gain_mode_disabled():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={"kp": 20.0, "kd": 3.0, "gain_mode": 0},
        j_eff_by_motor={2: 0.5},
        adaptive_state=None,
    )
    assert (kp, kd, j_eff) == (20.0, 3.0, None)


def test_adaptive_gain_for_motor_computes_gain_from_j_eff_when_enabled():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={"kp": 20.0, "kd": 3.0, "gain_mode": 1, "omega_n_target": 6.0, "zeta_target": 1.25},
        j_eff_by_motor={2: 0.5},
        adaptive_state=None,
    )
    assert (kp, kd, j_eff) == (18.0, 7.5, 0.5)


def test_adaptive_gain_for_motor_falls_back_to_fixed_gain_when_j_eff_missing():
    kp, kd, j_eff = _adaptive_gain_for_motor(
        motor_id=2,
        tuning={"kp": 20.0, "kd": 3.0, "gain_mode": 1, "omega_n_target": 6.0, "zeta_target": 1.25},
        j_eff_by_motor={},
        adaptive_state=None,
    )
    assert (kp, kd, j_eff) == (20.0, 3.0, None)
```

- [ ] **Step 2: Run tests to verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan_node_gain_scaling.py::test_adaptive_gain_for_motor_computes_gain_from_j_eff_when_enabled -q
```

Expected: import failure because `_adaptive_gain_for_motor` does not exist.

- [ ] **Step 3: Implement helper and state**

In `plan_node.py`, import:

```python
from phy.adaptive_gain import AdaptiveGainState, compute_adaptive_gains, lpf_j_eff
```

Add helper near other top-level helpers:

```python
def _adaptive_gain_for_motor(
    *,
    motor_id: int,
    tuning: dict,
    j_eff_by_motor: dict[int, float],
    adaptive_state: AdaptiveGainState | None,
) -> tuple[float, float, float | None]:
    fixed_kp = float(tuning.get("kp", 0.0))
    fixed_kd = float(tuning.get("kd", 0.0))
    if int(tuning.get("gain_mode", 0)) != 1:
        return fixed_kp, fixed_kd, None
    raw_j_eff = j_eff_by_motor.get(int(motor_id))
    if raw_j_eff is None:
        return fixed_kp, fixed_kd, None
    alpha = tuning.get("gain_lpf_alpha")
    j_eff = (
        lpf_j_eff(adaptive_state, motor_id=int(motor_id), raw_j_eff=float(raw_j_eff), alpha=alpha)
        if adaptive_state is not None
        else float(raw_j_eff)
    )
    if j_eff is None:
        return fixed_kp, fixed_kd, None
    gains = compute_adaptive_gains(
        j_eff,
        float(tuning.get("omega_n_target", 0.0)),
        float(tuning.get("zeta_target", 0.0)),
    )
    if gains is None:
        return fixed_kp, fixed_kd, None
    return gains[0], gains[1], float(j_eff)
```

In `PlanNode.__init__`, add:

```python
self._adaptive_gain_state = AdaptiveGainState()
```

Add a private method to compute `j_eff_by_motor` from a q dict:

```python
def _j_eff_by_motor(self, q_by_motor: dict[int, float]) -> dict[int, float]:
    try:
        diag = np.diag(self.robot.mass_matrix(q_by_motor))
    except Exception as exc:
        self.get_logger().warn(f"adaptive gain mass matrix failed: {exc}")
        return {}
    return {m: float(diag[i]) for i, m in enumerate(self.motor_ids)}
```

In both `_trajectory_cmds()` and `_hold_cmds()`, compute `j_eff_by_motor` once and replace fixed `kp/kd` reads with `_adaptive_gain_for_motor(...)`. Store `"j_eff": j_eff if j_eff is not None else math.nan` in outgoing command values.

- [ ] **Step 4: Run tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/phy/test/test_plan_node_gain_scaling.py -q
```

Expected: pass.

### Task 5: Extend Offline Safety Checker

**Files:**
- Modify: `motor/control_param_check.py`
- Test: `motor/tests/test_control_param_check_adaptive.py`

- [ ] **Step 1: Write failing tests for pure adaptive validation helpers**

Create `motor/tests/test_control_param_check_adaptive.py`:

```python
import importlib.util
from pathlib import Path


def _load_module():
    path = Path(__file__).resolve().parents[1] / "control_param_check.py"
    spec = importlib.util.spec_from_file_location("control_param_check", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_adaptive_gain_samples_report_clamp_and_effective_zeta_drop():
    mod = _load_module()

    rows = mod._adaptive_gain_rows(
        j_values=[0.5],
        omega_n_target=10.0,
        zeta_target=1.0,
        kp_max=50.0,
        kd_max=5.0,
        qd_noise_rms=0.09,
        tau_noise_budget=0.4,
    )

    assert rows[0]["kp"] == 50.0
    assert rows[0]["kd"] == 10.0
    assert rows[0]["kp_clamped"] == 50.0
    assert rows[0]["kd_clamped"] == 5.0
    assert rows[0]["zeta_eff"] == 0.5
    assert rows[0]["kd_noise_tau"] == 0.9


def test_adaptive_gain_validation_rejects_underdamped_target():
    mod = _load_module()

    errors = []
    warns = []
    mod._validate_adaptive_gain_config(
        cid=2,
        tuning={"gain_mode": 1, "omega_n_target": 6.0, "zeta_target": 0.7},
        j_values=[0.2],
        kp_max=60.0,
        kd_max=10.0,
        qd_noise_rms=0.09,
        tau_noise_budget=0.4,
        errors=errors,
        warns=warns,
    )

    assert any("zeta_target=0.7 < 1.0" in e for e in errors)
```

- [ ] **Step 2: Run tests to verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest motor/tests/test_control_param_check_adaptive.py -q
```

Expected: helper functions missing.

- [ ] **Step 3: Implement helper functions and CLI integration**

Add arguments:

```python
ap.add_argument("--qd_noise_rms", type=float, default=0.09)
ap.add_argument("--tau_noise_budget", type=float, default=0.4)
```

Add helpers:

```python
def _adaptive_gain_rows(...): ...
def _validate_adaptive_gain_config(...): ...
```

In `main()`, for `gain_mode=1`, validate `omega_n_target`, `zeta_target`, adaptive clamp, kd noise budget, and clamped effective zeta. Print adaptive mode values in flags and append ERRORs on violations.

- [ ] **Step 4: Run tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest motor/tests/test_control_param_check_adaptive.py -q
```

Expected: pass.

### Task 6: Full Verification

**Files:**
- All modified files.

- [ ] **Step 1: Run focused pytest suite**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/phy/test/test_adaptive_gain.py \
  src/phy/test/test_plan_node_gain_scaling.py \
  motor/tests/test_control_tuning_workflow.py \
  motor/tests/test_control_param_check_adaptive.py \
  -q
```

Expected: pass.

- [ ] **Step 2: Run offline checker smoke test**

Run:

```bash
cd /home/su/idle_ws/motor && PYTHONPATH=/home/su/idle_ws/motor python3 control_param_check.py --can_id 1 2 --kp_max 60 --kd_max 10
```

Expected: script runs and reports pass or existing tuning warnings/errors. If it exits nonzero because current tuned fixed gains violate existing checker policy, record the output; do not weaken safety checks.

- [ ] **Step 3: Inspect diff**

Run:

```bash
git diff -- src/phy/phy/adaptive_gain.py src/phy/phy/plan_node.py src/idle_common/idle_common/control_tuning.py motor/control_param_check.py src/phy/test/test_adaptive_gain.py src/phy/test/test_plan_node_gain_scaling.py motor/tests/test_control_tuning_workflow.py motor/tests/test_control_param_check_adaptive.py
```

Expected: only scoped adaptive gain scheduling changes.
