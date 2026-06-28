"""Small validation helpers for task-level commands."""

from __future__ import annotations


def is_known_task(task: str, presets: dict) -> bool:
    normalized = (task or "").strip()
    if normalized in ("", "default"):
        return True
    return normalized in presets
