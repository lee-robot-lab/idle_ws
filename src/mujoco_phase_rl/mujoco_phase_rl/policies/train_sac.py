from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="Train SAC on IdlePhasePickPlace-v0.")
    parser.add_argument("--total-timesteps", type=int, default=200_000)
    parser.parse_args()
    try:
        import stable_baselines3  # noqa: F401
        import gymnasium  # noqa: F401
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "train_sac requires gymnasium and stable-baselines3. "
            "Install those dependencies after PPO is stable."
        ) from exc
    raise SystemExit("SAC training wiring is reserved for Milestone 7.")


if __name__ == "__main__":
    main()
