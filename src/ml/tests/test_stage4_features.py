import math

import torch

from stage4.features import anchor_features_from_label, normalized_xy_to_world, normalized_xy_yaw_to_world_yaw


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


def test_normalized_xy_yaw_to_world_yaw_identity_preserves_yaw():
    xy = torch.tensor([[0.5, 0.5]], dtype=torch.float32)
    yaw = torch.tensor([math.pi / 6], dtype=torch.float32)
    H = torch.eye(3, dtype=torch.float32)

    world_yaw = normalized_xy_yaw_to_world_yaw(xy, yaw, H)

    assert torch.allclose(world_yaw, yaw, atol=1e-5)


def test_normalized_xy_yaw_to_world_yaw_accounts_for_homography_axis_flip():
    xy = torch.tensor([[0.5, 0.5]], dtype=torch.float32)
    yaw = torch.tensor([math.pi / 4], dtype=torch.float32)
    H = torch.tensor(
        [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=torch.float32,
    )

    world_yaw = normalized_xy_yaw_to_world_yaw(xy, yaw, H)

    assert torch.allclose(world_yaw, torch.tensor([-math.pi / 4]), atol=1e-5)
