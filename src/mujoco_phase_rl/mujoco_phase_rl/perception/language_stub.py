from __future__ import annotations

import numpy as np


def language_task_embedding(task_id: int = 0, size: int = 8) -> np.ndarray:
    embedding = np.zeros(size, dtype=np.float32)
    if 0 <= task_id < size:
        embedding[task_id] = 1.0
    return embedding
