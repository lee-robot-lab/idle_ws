import torch
import inspect

from stage4.model import RelationScorer


def test_relation_scorer_forward_shape_and_invalid_mask():
    model = RelationScorer()
    slots = torch.randn(2, 6, 256)
    color_logits = torch.randn(2, 6, 4)
    world_xy = torch.randn(2, 6, 2)
    yaw = torch.randn(2, 6, 2)
    relation_id = torch.tensor([0, 1])
    query_kind_id = torch.tensor([0, 1])
    phase_id = torch.tensor([0, 1])
    anchor = torch.randn(2, model.anchor_dim)
    valid_mask = torch.tensor(
        [
            [True, True, False, True, False, False],
            [False, True, True, False, False, False],
        ]
    )

    logits = model(
        slots=slots,
        color_logits=color_logits,
        world_xy=world_xy,
        yaw=yaw,
        relation_id=relation_id,
        query_kind_id=query_kind_id,
        phase_id=phase_id,
        anchor_features=anchor,
        valid_mask=valid_mask,
    )

    assert logits.shape == (2, 6)
    assert torch.isneginf(logits[0, 2])
    assert torch.isneginf(logits[1, 0])


def test_relation_scorer_exposes_no_geometric_score_argument():
    model = RelationScorer()
    sig = inspect.signature(model.forward)
    names = set(sig.parameters)
    forbidden = {"geometric_score", "geometry_score", "lambda_geo", "geo_score"}
    assert names.isdisjoint(forbidden)


def test_relation_scorer_handles_all_invalid_candidates():
    model = RelationScorer()
    logits = model(
        slots=torch.randn(1, 6, 256),
        color_logits=torch.randn(1, 6, 4),
        world_xy=torch.randn(1, 6, 2),
        yaw=torch.randn(1, 6, 2),
        relation_id=torch.tensor([0]),
        query_kind_id=torch.tensor([0]),
        phase_id=torch.tensor([0]),
        anchor_features=torch.randn(1, model.anchor_dim),
        valid_mask=torch.zeros(1, 6, dtype=torch.bool),
    )
    assert torch.isneginf(logits).all()
