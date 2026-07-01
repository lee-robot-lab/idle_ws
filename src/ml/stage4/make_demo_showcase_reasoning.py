# ================================================================
# stage4/make_demo_showcase_reasoning.py
# 설명: Stage4 RelationScorer의 self/cross-attention 가중치를 실제 추론에서 뽑아
#       관계어 grounding 예측 + attention 히트맵 + 정확도를 한 장으로 만든다.
# 사용법:
#   python src/ml/stage4/make_demo_showcase_reasoning.py
# ================================================================
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "Noto Sans CJK JP"
import numpy as np
import torch

_ML_ROOT = Path(__file__).resolve().parents[1]
if str(_ML_ROOT) not in sys.path:
    sys.path.insert(0, str(_ML_ROOT))

from stage1.dataset import CROP_H, CROP_W, CROP_X0, CROP_Y0, _MEAN, _STD
from stage1.model_dn import SlotEncoderDN
from stage2.color_net_v2 import ColorNetV2
from stage4.build_labels import generate_scene_samples, load_scene_labels
from stage4.constants import COLOR_TO_ID, ID_TO_COLOR, PHASE_TO_ID, QUERY_KIND_TO_ID, RELATION_TO_ID
from stage4.features import anchor_features_from_label, normalized_xy_to_world
from stage4.grounding import valid_candidate_mask
from stage4.model import RelationScorer

ROOT = _ML_ROOT.parents[1]
SCENES_DIR = ROOT / "data" / "scenes"
SPLIT_JSON = ROOT / "data" / "split.json"
STAGE1_CKPT = ROOT / "checkpoints" / "stage1_v2" / "best.pt"
COLOR_NET_CKPT = ROOT / "checkpoints" / "color_net_v2" / "best.pt"
STAGE4_CKPT = ROOT / "checkpoints" / "stage4" / "best.pt"
OUT_PATH = ROOT / "viz" / "demo_showcase" / "reasoning.png"

PRESENT_THR = 0.5
MIN_VALID_SLOTS = 3

_COLOR_SHORT = {"red_block": "red", "green_block": "green", "blue_block": "blue", "basket": "basket"}
_PALETTE_BGR = {0: (0, 0, 255), 1: (0, 180, 0), 2: (255, 0, 0), 3: (0, 255, 255)}
GT_BGR = (255, 255, 255)
PRED_BGR = (255, 0, 255)


def load_scene_tensor(scene_id: str):
    img = cv2.imread(str(SCENES_DIR / f"{scene_id}.jpg"))
    crop = img[CROP_Y0:, CROP_X0 : CROP_X0 + CROP_W]
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (416, 288))
    norm = (resized.astype(np.float32) / 255.0 - _MEAN) / _STD
    tensor = torch.from_numpy(norm.transpose(2, 0, 1)).unsqueeze(0).float()
    return crop, tensor


@torch.no_grad()
def forward_with_attention(model: RelationScorer, **kw):
    """RelationScorer.forward를 그대로 복제하되 self/cross-attention 가중치도 반환한다."""
    color_prob = torch.softmax(kw["color_logits"], dim=-1)
    slot_tokens = (
        model.slot_proj(kw["slots"])
        + model.color_proj(color_prob)
        + model.xy_proj(kw["world_xy"])
        + model.yaw_proj(kw["yaw"])
    )
    slot_tokens = model.norm_slots(slot_tokens)
    sa_mask = ~kw["valid_mask"].bool()
    sa_out, sa_weights = model.slot_self_attn(
        slot_tokens, slot_tokens, slot_tokens, key_padding_mask=sa_mask,
        need_weights=True, average_attn_weights=True,
    )
    slot_tokens = model.norm_slots_sa(slot_tokens + sa_out)

    query = (
        model.relation_emb(kw["relation_id"])
        + model.query_kind_emb(kw["query_kind_id"])
        + model.phase_emb(kw["phase_id"])
        + model.anchor_proj(kw["anchor_features"])
    ).unsqueeze(1)

    key_padding_mask = ~kw["valid_mask"].bool()
    cross_weights = []
    for attn, norm in zip(model.attn, model.norm_q):
        update, w = attn(
            query, slot_tokens, slot_tokens,
            key_padding_mask=key_padding_mask, need_weights=True, average_attn_weights=True,
        )
        query = norm(query + update)
        cross_weights.append(w)

    context = query.expand(-1, slot_tokens.shape[1], -1)
    logits = model.scorer(torch.cat([slot_tokens, context], dim=-1)).squeeze(-1)
    logits = logits.masked_fill(~kw["valid_mask"].bool(), float("-inf"))
    return logits, sa_weights, cross_weights


def build_inputs(scene_id, sample, encoder, color_net, device):
    crop, x = load_scene_tensor(scene_id)
    x = x.to(device)
    with torch.no_grad():
        out = encoder(x)
        present_mask = torch.sigmoid(out["present"].squeeze(-1)) > PRESENT_THR
        color_logits, _ = color_net(x, out["xy"])
        slot_to_color = color_net.assign(color_logits[0], present_mask[0]).unsqueeze(0)
        valid_mask = valid_candidate_mask(slot_to_color[0], present_mask[0], query_type="block").unsqueeze(0)
        world_xy = normalized_xy_to_world(out["xy"])

    labels = load_scene_labels(SCENES_DIR, scene_id)
    reference = sample["reference"]
    anchor_label = labels.get(reference) if reference not in (None, "robot") else None
    anchor_features = anchor_features_from_label(anchor_label, reference).unsqueeze(0).to(device)

    return {
        "crop": crop,
        "xy": out["xy"][0],
        "slots": out["slots"],
        "color_logits": color_logits,
        "world_xy": world_xy,
        "yaw": out["yaw"],
        "slot_to_color": slot_to_color[0],
        "valid_mask": valid_mask,
        "relation_id": torch.tensor([RELATION_TO_ID[sample["relation"]]], device=device),
        "query_kind_id": torch.tensor([QUERY_KIND_TO_ID[sample["query_kind"]]], device=device),
        "phase_id": torch.tensor([PHASE_TO_ID[sample["phase"]]], device=device),
        "anchor_features": anchor_features,
        "labels": labels,
    }


def pick_example(encoder, color_net, model, device):
    """정답을 맞춘 예시들 중 마지막 cross-attention layer가 정답 슬롯에 가장 뚜렷하게
    쏠리는(1등-2등 마진이 큰) 예시를 골라 attention 스토리가 흐릿하지 않게 한다."""
    split = json.loads(SPLIT_JSON.read_text())
    scene_ids = list(split["val"])

    best = None
    best_margin = -1.0
    for scene_id in scene_ids:
        labels = load_scene_labels(SCENES_DIR, scene_id)
        samples = generate_scene_samples(scene_id, labels, "OBJECT_QUERY")
        for sample in samples:
            inp = build_inputs(scene_id, sample, encoder, color_net, device)
            valid_idx = inp["valid_mask"][0].nonzero(as_tuple=True)[0]
            if len(valid_idx) < MIN_VALID_SLOTS:
                continue
            logits, sa_w, cross_w = forward_with_attention(model, **inp)
            pred_idx = int(torch.argmax(logits[0]).item())
            pred_object = ID_TO_COLOR[int(inp["slot_to_color"][pred_idx].item())]
            if pred_object != sample["target_object"]:
                continue

            valid_list = valid_idx.tolist()
            pred_pos = valid_list.index(pred_idx)
            last_layer_w = cross_w[-1][0, 0][valid_idx]
            sorted_w, sorted_pos = torch.sort(last_layer_w, descending=True)
            if int(sorted_pos[0].item()) != pred_pos:
                continue  # 마지막 layer가 정답 슬롯에 쏠리지 않는 예시는 스토리가 헷갈리므로 제외
            margin = float((sorted_w[0] - sorted_w[1]).item()) if len(sorted_w) > 1 else float(sorted_w[0].item())
            if margin > best_margin:
                best_margin = margin
                best = (scene_id, sample, inp, logits, sa_w, cross_w)

    if best is None:
        raise RuntimeError("no correct example found with enough valid slots")
    return best


ATTN_CMAP = "viridis"


def _text_color(value, vmax):
    """cmap 밝기에 따라 대비되는 텍스트 색 선택 (진한 칸=흰 글자, 옅은 칸=검은 글자)."""
    rgba = plt.get_cmap(ATTN_CMAP)(0.0 if vmax <= 0 else value / vmax)
    luminance = 0.299 * rgba[0] + 0.587 * rgba[1] + 0.114 * rgba[2]
    return "black" if luminance > 0.6 else "white"


def _tight_crop_bounds(labels: dict, margin_px: int = 150):
    xs = [l["center_px"][0] for l in labels.values()]
    ys = [l["center_px"][1] for l in labels.values()]
    x0 = max(CROP_X0, int(min(xs) - margin_px))
    x1 = min(CROP_X0 + CROP_W, int(max(xs) + margin_px))
    y0 = max(CROP_Y0, int(min(ys) - margin_px))
    y1 = min(CROP_Y0 + CROP_H, int(max(ys) + margin_px))
    return x0, y0, x1, y1


def draw_scene_panel(ax, scene_id, sample, inp, logits):
    crop = inp["crop"].copy()
    labels = inp["labels"]

    for name, label in labels.items():
        u, v = label["center_px"]
        u, v = int(round(u)) - CROP_X0, int(round(v)) - CROP_Y0
        cv2.circle(crop, (u, v), 9, (210, 210, 210), 2)

    def label_right(cx, cy, ring_r, text, color, dy):
        """마커 오른쪽 바깥에 라벨 배치 (링과 겹치지 않도록)."""
        tx, ty = cx + ring_r + 10, cy + dy
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(crop, (tx - 3, ty - th - 3), (tx + tw + 3, ty + 5), (0, 0, 0), -1)
        cv2.putText(crop, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

    target = sample["target_object"]
    gu, gv = labels[target]["center_px"]
    gu, gv = int(round(gu)) - CROP_X0, int(round(gv)) - CROP_Y0
    cv2.circle(crop, (gu, gv), 16, GT_BGR, 4)

    pred_idx = int(torch.argmax(logits[0]).item())
    xy = inp["xy"][pred_idx]
    pu = int(round(xy[0].item() * CROP_W))
    pv = int(round(xy[1].item() * CROP_H))
    cv2.circle(crop, (pu, pv), 24, PRED_BGR, 4)

    label_right(gu, gv, 16, "GT", GT_BGR, dy=-6)
    label_right(pu, pv, 24, "PRED", PRED_BGR, dy=26)

    x0, y0, x1, y1 = _tight_crop_bounds(labels)
    crop = crop[y0 - CROP_Y0 : y1 - CROP_Y0, x0 - CROP_X0 : x1 - CROP_X0]

    ax.imshow(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
    ref = sample["reference"]
    query_str = f"{sample['relation']}({_COLOR_SHORT.get(ref, ref)})" if ref else sample["relation"]
    pred_object = ID_TO_COLOR[int(inp["slot_to_color"][pred_idx].item())]
    ok = pred_object == target
    status_color = "green" if ok else "red"
    ax.set_title(
        f"{scene_id}   query: {query_str}\n"
        f"GT={_COLOR_SHORT.get(target, target)}   Pred={_COLOR_SHORT.get(pred_object, pred_object)}   "
        f"[{'OK' if ok else 'WRONG'}]",
        fontsize=13, color=status_color, fontweight="bold",
    )
    ax.axis("off")


def slot_labels_for(inp, valid_idx):
    return [_COLOR_SHORT.get(ID_TO_COLOR[int(inp["slot_to_color"][i].item())], "?") for i in valid_idx]


def draw_self_attention_panel(ax, inp, sa_weights, valid_idx):
    mat = sa_weights[0][valid_idx][:, valid_idx].cpu().numpy()
    labels = slot_labels_for(inp, valid_idx)
    im = ax.imshow(mat, cmap=ATTN_CMAP, vmin=0, vmax=1)
    ax.set_xticks(range(len(labels)))
    ax.set_yticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_yticklabels(labels, fontsize=11)
    for i in range(len(labels)):
        for j in range(len(labels)):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center",
                     fontsize=11, color=_text_color(mat[i, j], 1))
    ax.set_xlabel("Key 슬롯 (참조 대상)", fontsize=10)
    ax.set_ylabel("Query 슬롯 (기준 슬롯)", fontsize=10)
    ax.set_title("① Self-Attention (슬롯 ↔ 슬롯)\n행=Query, 열=Key, 값=행이 열에 주는 attention", fontsize=11)
    plt.colorbar(im, ax=ax, fraction=0.046)


def draw_cross_attention_panel(ax, inp, cross_weights, valid_idx):
    labels = slot_labels_for(inp, valid_idx)
    n_layers = len(cross_weights)
    mat = np.stack([w[0, 0, valid_idx].cpu().numpy() for w in cross_weights], axis=0)  # (layers, n_valid)
    im = ax.imshow(mat, cmap=ATTN_CMAP, vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_yticks(range(n_layers))
    ax.set_yticklabels([f"Query\n(layer{i} 이후)" for i in range(n_layers)], fontsize=10)
    for i in range(n_layers):
        for j in range(len(labels)):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center",
                     fontsize=11, color=_text_color(mat[i, j], 1))
    ax.set_xlabel("Key 슬롯 (명령이 훑는 대상)", fontsize=10)
    ax.set_title("② Cross-Attention (명령 토큰 → 슬롯)\n행=Query(명령 토큰, layer별), 열=Key(슬롯)", fontsize=11)
    plt.colorbar(im, ax=ax, fraction=0.046)


def draw_accuracy_panel(ax, metrics):
    per_rel = metrics["per_relation"]
    names = list(per_rel.keys())
    values = [per_rel[k] * 100 for k in names]
    bars = ax.bar(names, values, color="steelblue")
    ax.axhline(metrics["accuracy"] * 100, color="red", linestyle="--", linewidth=1,
               label=f"overall {metrics['accuracy'] * 100:.1f}%")
    ax.set_ylim(0, 112)
    ax.set_ylabel("val accuracy (%)", fontsize=10)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=30, ha="right", fontsize=9)
    ax.legend(fontsize=9, loc="lower right")
    for bar, v in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 2, f"{v:.1f}", ha="center", fontsize=8)
    ax.set_title("Stage4 RelationScorer — 관계별 val accuracy", fontsize=12)


def draw_architecture_panel(ax):
    """토큰 구성(positional encoding 포함) → self-attn → cross-attn 흐름을 좌→우로 넓게 그린다."""
    ax.set_xlim(0, 17.2)
    ax.set_ylim(0, 8.6)
    ax.axis("off")

    def box(xy, w, h, text, fc="#e8eef7", ec="#333333", fontsize=9.5, fontweight="normal", z=3):
        rect = plt.matplotlib.patches.FancyBboxPatch(
            xy, w, h, boxstyle="round,pad=0.06,rounding_size=0.12",
            linewidth=1.4, edgecolor=ec, facecolor=fc, zorder=z,
        )
        ax.add_patch(rect)
        ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha="center", va="center",
                 fontsize=fontsize, fontweight=fontweight, zorder=z + 1)
        return xy[0] + w / 2, xy[1], xy[1] + h

    def arrow(x0, y0, x1, y1, color="#555555", lw=1.6):
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                     arrowprops=dict(arrowstyle="-|>", color=color, lw=lw), zorder=2)

    def zone(x0, x1, label, fc):
        ax.add_patch(plt.matplotlib.patches.FancyBboxPatch(
            (x0, 0.1), x1 - x0, 8.4, boxstyle="round,pad=0.02,rounding_size=0.15",
            linewidth=1.2, linestyle="--", edgecolor="#999999", facecolor=fc, zorder=0,
        ))
        ax.text(x0 + 0.15, 8.15, label, fontsize=10, fontweight="bold", color="#666666", va="top")

    zone(0.0, 7.0, "토큰 구성 (슬롯 1개당)", "#f7f7f7")
    zone(7.1, 17.2, "Attention 추론", "#f5faff")

    # ── 입력 4종 (슬롯 토큰 구성 요소), 왼쪽 열에 세로로 쌓음 ──────────
    box((0.3, 6.3), 2.6, 1.1, "Slot embedding (256d)\nStage1 decoder output", fontsize=8.3)
    box((0.3, 4.9), 2.6, 1.1, "Color prob (4d)\nStage2 ColorNet", fontsize=8.3)

    uv_x, uv_y0, uv_y1 = box((0.3, 3.75), 2.6, 0.55, "Stage1 예측 uv (crop 0~1)", fontsize=7.5)
    wx_x, wx_y0, wx_y1 = box((0.3, 2.6), 2.6, 0.85, "world xy (m)\nPositional Encoding (MLP)",
                              fc="#ffe1b3", ec="#c67c00", fontsize=7.8, fontweight="bold")
    arrow(uv_x, uv_y0, wx_x, wx_y1, color="#c67c00")
    ax.text(uv_x + 1.35, (uv_y0 + wx_y1) / 2, "homography", fontsize=7, color="#c67c00", style="italic")

    box((0.3, 1.1), 2.6, 1.1, "yaw (cos4θ, sin4θ)\nStage1 head", fontsize=8.3)

    input_mids = [6.3 + 0.55, 4.9 + 0.55, (wx_y0 + wx_y1) / 2, 1.1 + 0.55]
    center_y = sum(input_mids) / len(input_mids)

    sum_x, sum_y0, sum_y1 = box((3.3, center_y - 1.5), 1.1, 3.0, "sum\n(+)", fc="#dddddd")
    for cy in input_mids:
        arrow(2.9, cy, 3.3, center_y)

    tok_x, tok_y0, tok_y1 = box((4.8, center_y - 1.5), 2.0, 3.0, "slot token\n(per slot)")
    arrow(sum_x + 0.55, center_y, 4.8, center_y)

    # ── Attention 추론 존 ─────────────────────────────────────────
    sa_x, sa_y0, sa_y1 = box((7.5, 4.6), 3.0, 1.7, "① Self-Attention\nQuery=Key=슬롯\n(슬롯 ↔ 슬롯)",
                              fc="#d7ecff", ec="#1f6fb2", fontsize=9)
    arrow(6.8, center_y, 7.5, (sa_y0 + sa_y1) / 2)

    q_x, q_y0, q_y1 = box((7.5, 1.0), 3.0, 1.7, "query token\nrelation+kind+phase+anchor", fontsize=8.6)

    ca_x, ca_y0, ca_y1 = box((11.2, 2.8), 3.0, 2.1,
                              "② Cross-Attention\nQuery=명령 토큰, Key=슬롯\n(2 layers)",
                              fc="#ffd9e8", ec="#b21f6f", fontsize=9)
    arrow(sa_x + 1.5, sa_y0, 11.2, ca_y1 - 0.4)
    arrow(q_x + 1.5, q_y1, 11.2, ca_y0 + 0.5)

    box((14.9, 3.2), 1.3, 1.3, "슬롯별\nscore", fontsize=8.5)
    arrow(ca_x + 1.5, (ca_y0 + ca_y1) / 2, 14.9, 3.85)

    ax.set_title("Stage4 토큰 구성 & attention 파이프라인", fontsize=13)


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    encoder = SlotEncoderDN(num_queries=6, dec_layers=3, dino_dim=384).to(device).eval()
    encoder.load_state_dict(torch.load(STAGE1_CKPT, map_location="cpu", weights_only=False)["state_dict"])

    color_net = ColorNetV2().to(device).eval()
    color_net.load_state_dict(torch.load(COLOR_NET_CKPT, map_location="cpu", weights_only=False)["color_net"])

    stage4_ckpt = torch.load(STAGE4_CKPT, map_location="cpu", weights_only=False)
    model = RelationScorer().to(device).eval()
    model.load_state_dict(stage4_ckpt["model"])
    metrics = stage4_ckpt["metrics"]

    scene_id, sample, inp, logits, sa_weights, cross_weights = pick_example(encoder, color_net, model, device)
    valid_idx = inp["valid_mask"][0].nonzero(as_tuple=True)[0].cpu()

    fig = plt.figure(figsize=(22, 16))
    gs = fig.add_gridspec(3, 3, height_ratios=[1.3, 0.95, 1.0])
    ax_scene = fig.add_subplot(gs[0, :])
    ax_arch = fig.add_subplot(gs[1, :])
    ax_self = fig.add_subplot(gs[2, 0])
    ax_cross = fig.add_subplot(gs[2, 1])
    ax_acc = fig.add_subplot(gs[2, 2])

    draw_scene_panel(ax_scene, scene_id, sample, inp, logits)
    draw_architecture_panel(ax_arch)
    draw_self_attention_panel(ax_self, inp, sa_weights, valid_idx)
    draw_cross_attention_panel(ax_cross, inp, cross_weights, valid_idx)
    draw_accuracy_panel(ax_acc, metrics)

    fig.suptitle(
        "Stage4 — slot 임베딩에 좌표 positional encoding을 더한 토큰으로 "
        "self/cross-attention 기반 관계어 grounding",
        fontsize=15,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.subplots_adjust(hspace=0.35, wspace=0.3)

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_PATH, dpi=150)
    print(f"saved: {OUT_PATH}  (scene={scene_id}, query={sample['relation']}({sample['reference']}))")


if __name__ == "__main__":
    main()
