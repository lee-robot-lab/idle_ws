# ================================================================
# dataset/add_target_colors.py
# 설명: 기존 scene JSON에 target_colors 필드를 일괄 추가한다.
# 사용법: python dataset/add_target_colors.py --scenes_dir data/scenes
# ================================================================
import argparse
import json
from pathlib import Path

COLORS = ["red", "green", "blue", "basket"]


def add_target_colors_to_dir(scenes_dir: str, default_colors: list[str] = COLORS) -> int:
    """JSON 파일에 target_colors 추가. 이미 있으면 건너뜀. 수정 파일 수 반환."""
    count = 0
    for p in Path(scenes_dir).glob("*.json"):
        data = json.loads(p.read_text())
        if "target_colors" in data:
            continue
        data["target_colors"] = [c for c in default_colors if c in data]
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        count += 1
    return count


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_dir", default="data/scenes")
    parser.add_argument("--colors", nargs="+", default=COLORS)
    args = parser.parse_args()
    n = add_target_colors_to_dir(args.scenes_dir, args.colors)
    print(f"Updated {n} files")
