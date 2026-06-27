import torch

from stage4.features import anchor_features_from_label, normalized_xy_to_world


def test_anchor_features_for_basket_has_fixed_width():
    features = anchor_features_from_label(
        {"x": 0.4, "y": 0.1, "cos_yaw": 1.0, "sin_yaw": 0.0},
        "basket",
    )
    assert features.shape == (16,)


def test_normalized_xy_to_world_shape():
    xy = torch.tensor([[[0.0, 0.0], [1.0, 1.0]]])
    world = normalized_xy_to_world(xy)
    assert world.shape == (1, 2, 2)
    assert torch.isfinite(world).all()
