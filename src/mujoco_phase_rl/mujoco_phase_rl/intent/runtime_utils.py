from __future__ import annotations

import importlib.util
import json
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import time
import tty
from pathlib import Path
from typing import Any

from mujoco_phase_rl.intent.task_router import (
    TaskRoute,
    load_route_config,
    parse_json_value,
    route_semantic_plan,
)


def workspace_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src" / "stt" / "stt.py").exists():
            return parent
    cwd = Path.cwd()
    if (cwd / "src" / "stt" / "stt.py").exists():
        return cwd
    return cwd


def default_stt_path() -> Path:
    return workspace_root() / "src" / "stt" / "stt.py"


def parse_intent_source(
    *,
    text: str | None,
    semantic_json: str | None,
    mic: bool = False,
    parser_mode: str,
    stt_path: str | Path,
    qwen_model: str,
    qwen_4bit: bool,
    mic_duration: float = 0.0,
    mic_backend: str = "auto",
    mic_empty_retries: int = 2,
    whisper_model: str = "small",
    whisper_device: str = "cpu",
    whisper_compute_type: str = "int8",
) -> dict[str, Any]:
    if mic:
        text = transcribe_microphone(
            stt_path=stt_path,
            duration_s=mic_duration,
            backend=mic_backend,
            empty_retries=mic_empty_retries,
            whisper_model=whisper_model,
            whisper_device=whisper_device,
            whisper_compute_type=whisper_compute_type,
        )
    if text:
        stt = _load_stt_module(stt_path)
        qwen_parser = None
        if parser_mode in {"qwen", "hybrid"}:
            qwen_parser = stt.QwenSemanticParser(qwen_model, use_4bit=qwen_4bit)
        return stt.parse_with_mode(text, parser_mode, qwen_parser)
    if semantic_json:
        return parse_json_value(semantic_json)
    raise ValueError("text, semantic_json, or mic is required")


def route_intent(
    *,
    text: str | None,
    semantic_json: str | None,
    mic: bool = False,
    parser_mode: str,
    stt_path: str | Path,
    qwen_model: str,
    qwen_4bit: bool,
    route_config: str | Path | None,
    mic_duration: float = 0.0,
    mic_backend: str = "auto",
    mic_empty_retries: int = 2,
    whisper_model: str = "small",
    whisper_device: str = "cpu",
    whisper_compute_type: str = "int8",
) -> tuple[dict[str, Any], TaskRoute]:
    plan = parse_intent_source(
        text=text,
        semantic_json=semantic_json,
        mic=mic,
        parser_mode=parser_mode,
        stt_path=stt_path,
        qwen_model=qwen_model,
        qwen_4bit=qwen_4bit,
        mic_duration=mic_duration,
        mic_backend=mic_backend,
        mic_empty_retries=mic_empty_retries,
        whisper_model=whisper_model,
        whisper_device=whisper_device,
        whisper_compute_type=whisper_compute_type,
    )
    route = route_semantic_plan(plan, config=load_route_config(route_config))
    return plan, route


def transcribe_microphone(
    *,
    stt_path: str | Path,
    duration_s: float = 0.0,
    backend: str = "auto",
    empty_retries: int = 2,
    whisper_model: str = "small",
    whisper_device: str = "cpu",
    whisper_compute_type: str = "int8",
) -> str:
    """Record one microphone clip and return Korean text via the STT package."""
    stt = _load_stt_module(stt_path)
    duration_s = float(duration_s)
    backend = backend.lower()
    if backend not in {"auto", "arecord", "sounddevice"}:
        raise ValueError(f"unsupported microphone backend: {backend}")

    use_arecord = backend in {"auto", "arecord"} and shutil.which("arecord") is not None
    if backend == "arecord" and not use_arecord:
        raise RuntimeError("arecord backend requires arecord")
    attempts = max(1, int(empty_retries) + 1)

    if use_arecord:
        from faster_whisper import WhisperModel

        model = WhisperModel(whisper_model, device=whisper_device, compute_type=whisper_compute_type)
        if duration_s > 0.0:
            recorder = lambda: _record_arecord_fixed_and_transcribe(stt, model, duration_s)
        else:
            recorder = lambda: _record_arecord_until_space_and_transcribe(stt, model)
    else:
        if not hasattr(stt, "load_voice_dependencies"):
            raise RuntimeError(f"STT module does not expose load_voice_dependencies: {stt_path}")
        stt.load_voice_dependencies()
        model = stt.WhisperModel(whisper_model, device=whisper_device, compute_type=whisper_compute_type)
        recorder = lambda: _record_sounddevice_and_transcribe(stt, model, duration_s)

    for attempt in range(1, attempts + 1):
        transcript = recorder().strip()
        if transcript:
            print(f"mic_transcript={transcript}", flush=True)
            return transcript
        if attempt < attempts:
            print(
                f"mic: transcript empty; retrying ({attempt}/{attempts - 1}). "
                "Speak clearly after recording starts.",
                file=sys.stderr,
                flush=True,
            )

    raise RuntimeError(
        "microphone transcript is empty after retries; check input device, mic gain, "
        "or run once with --text to bypass STT"
    )


def _record_arecord_fixed_and_transcribe(stt: Any, model: Any, duration_s: float) -> str:
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            temp_path = Path(tmp.name)
        duration_arg = str(max(1, int(round(duration_s))))
        print(f"mic: recording {duration_arg}s with arecord ...", flush=True)
        subprocess.run(
            [
                "arecord",
                "-q",
                "-f",
                "S16_LE",
                "-r",
                str(stt.fs),
                "-c",
                "1",
                "-d",
                duration_arg,
                str(temp_path),
            ],
            check=True,
        )
        return stt.transcribe_audio_file(model, str(temp_path)).strip()
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def _record_arecord_until_space_and_transcribe(stt: Any, model: Any) -> str:
    if not sys.stdin.isatty():
        raise RuntimeError("interactive microphone recording requires a terminal")

    temp_path: Path | None = None
    process: subprocess.Popen[str] | None = None
    stdin_fd = sys.stdin.fileno()
    original_terminal_settings = termios.tcgetattr(stdin_fd)
    started_at = 0.0
    failed = False
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            temp_path = Path(tmp.name)

        tty.setcbreak(stdin_fd)
        print("mic: press Space to start recording; press Space again to stop; q cancels", flush=True)
        while True:
            key = sys.stdin.read(1)
            if key.lower() == "q":
                raise RuntimeError("microphone recording cancelled")
            if key == " ":
                break

        print("mic: recording with arecord ...", flush=True)
        started_at = time.monotonic()
        process = subprocess.Popen(
            [
                "arecord",
                "-q",
                "-f",
                "S16_LE",
                "-r",
                str(stt.fs),
                "-c",
                "1",
                str(temp_path),
            ],
            text=True,
        )

        while True:
            key = sys.stdin.read(1)
            if key.lower() == "q":
                raise RuntimeError("microphone recording cancelled")
            if key == " " and time.monotonic() - started_at >= 0.35:
                break

    except BaseException:
        failed = True
        raise
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_terminal_settings)
        if process is not None and process.poll() is None:
            _stop_arecord(process)
        if failed and temp_path is not None and temp_path.exists():
            temp_path.unlink(missing_ok=True)

    if time.monotonic() - started_at < 0.2:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink(missing_ok=True)
        raise RuntimeError("microphone recording is too short")
    if temp_path is None or not temp_path.exists():
        raise RuntimeError("microphone wav was not created")

    try:
        return stt.transcribe_audio_file(model, str(temp_path)).strip()
    finally:
        temp_path.unlink(missing_ok=True)


def _stop_arecord(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)


def _record_sounddevice_and_transcribe(stt: Any, model: Any, duration_s: float) -> str:
    if duration_s > 0.0:
        sample_count = max(int(stt.fs * duration_s), int(stt.fs * 0.2))
        print(f"mic: recording {duration_s:.1f}s with sounddevice ...", flush=True)
        audio = stt.sd.rec(sample_count, samplerate=stt.fs, channels=1, dtype="float32")
        stt.sd.wait()
    else:
        audio = _record_microphone_until_space(stt)

    if audio is None or len(audio) < int(stt.fs * 0.2):
        raise RuntimeError("microphone recording is too short")

    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            temp_path = Path(tmp.name)
        stt.write(str(temp_path), stt.fs, audio)
        return stt.transcribe_audio_file(model, str(temp_path)).strip()
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def resolve_policy_model(
    route: TaskRoute,
    *,
    override: str | None = None,
    require_exists: bool = True,
) -> tuple[Path, str]:
    if override:
        path = Path(override).expanduser()
        if require_exists and not path.exists():
            raise FileNotFoundError(f"policy override does not exist: {path}")
        return path, "override"

    configured = Path(route.policy_model).expanduser()
    if configured.exists():
        return configured, "route_config"

    fallback_dir = _fallback_policy_dir(route)
    latest = latest_checkpoint(fallback_dir / "checkpoints")
    if latest is not None:
        return latest, f"latest_checkpoint:{fallback_dir}"

    final_model = fallback_dir / "final_model.zip"
    if final_model.exists():
        return final_model, f"fallback_final:{fallback_dir}"

    if require_exists:
        raise FileNotFoundError(
            "no policy model found. Checked route policy and fallback dir: "
            f"route={configured}, fallback={fallback_dir}"
        )
    return configured, "missing_route_config"


def latest_checkpoint(checkpoint_dir: str | Path) -> Path | None:
    root = Path(checkpoint_dir).expanduser()
    if not root.exists():
        return None
    pattern = re.compile(r"_(\d+)_steps\.zip$")
    candidates: list[tuple[int, Path]] = []
    for path in root.glob("*.zip"):
        match = pattern.search(path.name)
        if match:
            candidates.append((int(match.group(1)), path))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])[1]


def print_route_summary(plan: dict[str, Any], route: TaskRoute, policy_path: Path, source: str) -> None:
    step = plan["steps"][0]
    print("semantic")
    print(f"  action={step.get('action')} object={step.get('object')} target={step.get('target')}")
    print(f"  parser={plan.get('parser', '-')} reason={plan.get('reason', '-')}")
    print("route")
    print(f"  key={route.route_key} task_mode={route.task_mode}")
    print(f"  target_color={route.target_color} stack_target_color={route.stack_target_color or '-'}")
    print(f"  policy_model={policy_path} source={source}")


def _load_stt_module(stt_path: str | Path):
    path = Path(stt_path).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"STT parser not found: {path}")
    spec = importlib.util.spec_from_file_location("idle_stt_parser", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to import STT parser: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _record_microphone_until_space(stt: Any) -> Any:
    if not sys.stdin.isatty():
        raise RuntimeError("--mic-duration 0 requires an interactive terminal")
    stdin_fd = sys.stdin.fileno()
    original_terminal_settings = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        print("mic: press Space to start recording; press Space again to stop; q cancels", flush=True)
        while True:
            key = sys.stdin.read(1)
            if key.lower() == "q":
                raise RuntimeError("microphone recording cancelled")
            if key == " ":
                break
        audio, quit_requested = stt._record_until_space()
        if quit_requested:
            raise RuntimeError("microphone recording cancelled")
        return audio
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_terminal_settings)


def _fallback_policy_dir(route: TaskRoute) -> Path:
    if route.task_mode == "stack":
        return Path("outputs/stack_ppo_v1_staged_motion_1m")
    return Path("outputs/rgb_ppo_v2_staged_motion_robust_1m")


def dumps_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)
