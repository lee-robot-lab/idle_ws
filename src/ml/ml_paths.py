from __future__ import annotations

from pathlib import Path
from urllib.parse import urlparse


PACKAGE_NAME = "ml"


def package_share_dir(package_name: str = PACKAGE_NAME) -> Path:
    """Return an installed ament share path, or this source package root."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return Path(__file__).resolve().parent


def source_root() -> Path:
    """Return the source-tree root for direct `PYTHONPATH=src/ml` use."""
    return Path(__file__).resolve().parent


def workspace_root() -> Path:
    """Best-effort idle_ws root when running from the source tree."""
    root = source_root()
    if root.name == PACKAGE_NAME and root.parent.name == "src":
        return root.parent.parent
    return Path.cwd()


def data_root() -> Path:
    return workspace_root() / "data"


def checkpoint_root() -> Path:
    source_ckpt = source_root() / "checkpoints"
    if source_ckpt.exists():
        return source_ckpt
    return package_share_dir() / "checkpoints"


def resolve_path(value: str | Path, package_name: str = PACKAGE_NAME) -> Path:
    """Resolve filesystem, file://, and package:// paths for Python scripts.

    Supported examples:
      - /abs/path/model.pt
      - relative/path/model.pt
      - file:///abs/path/model.pt
      - package://ml/checkpoints/stage1_v2/best.pt
    """
    text = str(value)
    parsed = urlparse(text)
    if parsed.scheme == "file":
        return Path(parsed.path).expanduser()
    if parsed.scheme == "package":
        pkg = parsed.netloc or package_name
        rel = parsed.path.lstrip("/")
        return package_share_dir(pkg) / rel

    path = Path(text).expanduser()
    if path.is_absolute() or path.exists():
        return path

    source_candidate = source_root() / path
    if source_candidate.exists():
        return source_candidate

    share_candidate = package_share_dir(package_name) / path
    if share_candidate.exists():
        return share_candidate

    return path
