"""튜닝 파라미터 저장 및 dirty 해제를 수행."""

from lib.control_tuning import save_control_tuning


def main():
    path = save_control_tuning()
    print(f"saved control tuning: tuned_file={path} dirty=false")


if __name__ == "__main__":
    main()
