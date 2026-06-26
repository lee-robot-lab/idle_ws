# ================================================================
# dataset/crop.py
# 설명: 데이터 수집 후 이미지 일괄 crop (더티 데이터 방지용 고정 ROI).
# 사용법:
#   python crop.py --src raw/ --dst cropped/
#   python crop.py --src raw/          # dst 미지정 시 raw_cropped/ 에 저장
# ================================================================
import argparse
from pathlib import Path
import cv2

# 고정 ROI: x 90~1100, y 20~원래끝
X0, X1 = 90, 1100
Y0      = 20          # Y1은 원본 높이 그대로


def crop_images(src: Path, dst: Path) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    exts = {".jpg", ".jpeg", ".png"}
    files = sorted(p for p in src.iterdir() if p.suffix.lower() in exts)
    if not files:
        print(f"이미지 없음: {src}")
        return
    for p in files:
        img = cv2.imread(str(p))
        if img is None:
            print(f"  건너뜀(읽기 실패): {p.name}")
            continue
        cropped = img[Y0:, X0:X1]
        out = dst / p.name
        cv2.imwrite(str(out), cropped)
        print(f"  {p.name}  {img.shape[1]}x{img.shape[0]} → {cropped.shape[1]}x{cropped.shape[0]}")
    print(f"완료: {len(files)}장 → {dst}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, type=Path)
    ap.add_argument("--dst", type=Path)
    args = ap.parse_args()
    dst = args.dst if args.dst else args.src.parent / (args.src.name + "_cropped")
    crop_images(args.src, dst)


if __name__ == "__main__":
    main()
