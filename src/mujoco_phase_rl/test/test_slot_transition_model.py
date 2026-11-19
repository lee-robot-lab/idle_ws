import torch
from mujoco_phase_rl.world_model.slot_transition_model import SlotTransitionModel, compute_loss
from mujoco_phase_rl.world_model.dataset import X_DIM

B, T, H = 4, 6, 128


def test_forward_output_shapes():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    h_next, slot_pred, reward_pred, done_logit = model(x, h)
    assert h_next.shape == (B, H)
    assert slot_pred.shape == (B, 64)
    assert reward_pred.shape == (B, 1)
    assert done_logit.shape == (B, 1)


def test_rssm_latent_shape():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    h = model.init_hidden(B, torch.device("cpu"))
    latent = model.rssm_latent(h)
    assert latent.shape == (B, 64)


def test_init_hidden_zeros():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    h = model.init_hidden(B, torch.device("cpu"))
    assert h.shape == (B, H)
    assert torch.all(h == 0.0)


def test_compute_loss_returns_scalar_and_dict():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    _, slot_pred, reward_pred, done_logit = model(x, h)
    loss, info = compute_loss(
        slot_pred, torch.randn(B, 64),
        reward_pred, torch.randn(B, 1),
        done_logit, torch.zeros(B, 1),
    )
    assert loss.ndim == 0
    assert {"slot", "reward", "done"} == set(info.keys())


def test_sequence_unroll_no_error():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x_seq = torch.randn(B, T, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    for t in range(T):
        h, slot_pred, reward_pred, done_logit = model(x_seq[:, t], h)
    assert h.shape == (B, H)


def test_gradients_flow():
    model = SlotTransitionModel(input_dim=X_DIM, h_dim=H)
    x = torch.randn(B, X_DIM)
    h = model.init_hidden(B, torch.device("cpu"))
    h_next, slot_pred, reward_pred, done_logit = model(x, h)
    loss, _ = compute_loss(
        slot_pred, torch.randn(B, 64),
        reward_pred, torch.randn(B, 1),
        done_logit, torch.zeros(B, 1),
    )
    loss.backward()
    for name, param in model.named_parameters():
        if "rssm_latent_proj" in name:
            continue  # 학습 forward에서 사용 안 함 (RL obs 주입 시에만 호출)
        assert param.grad is not None, f"{name} has no gradient"
