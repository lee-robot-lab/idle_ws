from dataset.split import scene_level_split


def test_no_leakage_and_full_coverage():
    ids = [f"{i:06d}" for i in range(100)]
    sp = scene_level_split(ids, ratios=(0.7, 0.15, 0.15), seed=0)
    tr, va, te = set(sp["train"]), set(sp["val"]), set(sp["test"])
    assert tr & va == set() and tr & te == set() and va & te == set()
    assert tr | va | te == set(ids)


def test_deterministic_with_seed():
    ids = [f"{i:06d}" for i in range(50)]
    assert scene_level_split(ids, seed=42) == scene_level_split(ids, seed=42)


def test_approx_ratio():
    ids = [f"{i:06d}" for i in range(100)]
    sp = scene_level_split(ids, ratios=(0.7, 0.15, 0.15), seed=1)
    assert len(sp["train"]) == 70 and len(sp["val"]) == 15 and len(sp["test"]) == 15
