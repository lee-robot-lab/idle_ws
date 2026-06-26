# ================================================================
# labeling/relation.py
# 설명: world 좌표 기반 공간관계로 target 블록을 결정 (학습 라벨 생성용).
# 사용법: from labeling.relation import resolve_relation
# 주의: front_of/behind는 +y=전방 가정 (stage0 §3.4). 캘리 후 부호 검증.
# ================================================================
BLOCKS = ["red_block", "blue_block", "green_block"]


def _dist2(a, b):
    return (a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2


def _satisfies(block_xy, rel, ref_xy):
    r = rel["relation"]
    if r in ("nearest_to", "farthest_from", "leftmost", "rightmost"):
        return True  # 서열/거리로 tie-break 단계에서 처리
    bx, by = block_xy["x"], block_xy["y"]
    rx, ry = ref_xy["x"], ref_xy["y"]
    if r == "left_of":   return bx < rx
    if r == "right_of":  return bx > rx
    if r == "front_of":  return by > ry   # +y=전방
    if r == "behind":    return by < ry
    return False


def resolve_relation(scene_labels, relations):
    """AND 조건을 만족하는 block 이름. tie-break: 첫 relation 기준."""
    candidates = [b for b in BLOCKS if b in scene_labels and scene_labels[b] is not None]

    valid = []
    for b in candidates:
        ok = True
        for rel in relations:
            ref = rel["reference"]
            ref_xy = scene_labels[ref] if ref else None
            if not _satisfies(scene_labels[b], rel, ref_xy):
                ok = False
                break
        if ok:
            valid.append(b)
    if not valid:
        return None

    r0 = relations[0]
    ref = r0["reference"]
    rname = r0["relation"]
    if ref is not None:
        ref_xy = scene_labels[ref]
        reverse = (rname == "farthest_from")
        valid.sort(key=lambda b: _dist2(scene_labels[b], ref_xy), reverse=reverse)
    elif rname == "leftmost":
        valid.sort(key=lambda b: scene_labels[b]["x"])
    elif rname == "rightmost":
        valid.sort(key=lambda b: -scene_labels[b]["x"])
    return valid[0]
