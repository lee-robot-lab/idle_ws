"""튜닝 파라미터 저장 후 can_bridge Tx 정책 push를 수행."""

import argparse
import subprocess

from lib.control_tuning import (
    runtime_tx_policy_json_for_bridge,
    save_control_tuning,
)


def _push_tx_policy(bridge_node: str):
    tx_default, by_motor_json = runtime_tx_policy_json_for_bridge()
    subprocess.run(
        ["ros2", "param", "set", bridge_node, "tx_hz_default", f"{tx_default}"],
        check=True,
    )
    subprocess.run(
        ["ros2", "param", "set", bridge_node, "tx_hz_by_motor_json", by_motor_json],
        check=True,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge-node", default="/can_bridge_node")
    ap.add_argument(
        "--push",
        action="store_true",
        help="also push tx policy to can_bridge (default: save only, no push)",
    )
    ap.add_argument(
        "--skip-push",
        action="store_true",
        help="deprecated no-op; save is already no-push by default",
    )
    args = ap.parse_args()

    path = save_control_tuning()
    if not args.push:
        print(f"saved control tuning: tuned_file={path} dirty=false push=skipped")
        return

    try:
        _push_tx_policy(args.bridge_node)
    except FileNotFoundError:
        raise SystemExit("failed to push tx policy: 'ros2' command not found")
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f"failed to push tx policy to {args.bridge_node}: {exc}")
    print(
        "saved control tuning and pushed tx policy: "
        f"tuned_file={path} dirty=false bridge_node={args.bridge_node}"
    )


if __name__ == "__main__":
    main()
