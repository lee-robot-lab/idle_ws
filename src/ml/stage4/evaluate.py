from __future__ import annotations

from collections import defaultdict


def relation_accuracy(rows: list[dict]) -> dict:
    """Summarize rows with keys relation, pred, target."""
    correct = 0
    per_relation = defaultdict(lambda: [0, 0])
    for row in rows:
        ok = row["pred"] == row["target"]
        correct += int(ok)
        bucket = per_relation[row["relation"]]
        bucket[0] += int(ok)
        bucket[1] += 1
    total = len(rows)
    return {
        "accuracy": correct / total if total else 0.0,
        "per_relation": {
            k: v[0] / v[1] if v[1] else 0.0 for k, v in sorted(per_relation.items())
        },
    }
