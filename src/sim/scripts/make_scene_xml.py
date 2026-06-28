from __future__ import annotations

from pathlib import Path
import sys

_SIM_ROOT = Path(__file__).resolve().parents[1]
if str(_SIM_ROOT) not in sys.path:
    sys.path.insert(0, str(_SIM_ROOT))

from sim.scripts.make_scene_xml import main


if __name__ == "__main__":
    main()
