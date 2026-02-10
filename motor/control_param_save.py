from lib.control_tuning import save_control_tuning


def main():
    path = save_control_tuning()
    print(f"saved control tuning: tuned_file={path} dirty=false")


if __name__ == "__main__":
    main()
