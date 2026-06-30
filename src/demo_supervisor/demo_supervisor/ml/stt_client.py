# ================================================================
# demo_supervisor/ml/stt_client.py
# 설명: src/stt/stt.py를 subprocess로 호출해 텍스트 → semantic step dict 변환.
# ================================================================
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

_STT_SCRIPT = Path(__file__).resolve().parents[5] / "src" / "stt" / "stt.py"


def parse_text(text: str, parser: str = "rule") -> dict | None:
    """텍스트 → stt.py 파싱 → 첫 번째 step dict. 실패 시 None 반환."""
    try:
        result = subprocess.run(
            [sys.executable, str(_STT_SCRIPT), "--text", text, "--parser", parser],
            capture_output=True, text=True, timeout=30,
        )
        plan = json.loads(result.stdout)
        if not plan.get("success") or not plan.get("steps"):
            return None
        return plan["steps"][0]
    except Exception:
        return None
