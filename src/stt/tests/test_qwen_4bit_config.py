import argparse
import inspect
import pathlib
import sys


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import stt


def test_qwen_parser_uses_4bit_by_default():
    parser = stt.QwenSemanticParser()

    assert parser.use_4bit is True


def test_qwen_parser_can_disable_4bit():
    parser = stt.QwenSemanticParser(use_4bit=False)

    assert parser.use_4bit is False


def test_cli_exposes_boolean_qwen_4bit_flag():
    source = inspect.getsource(stt.main)

    assert "--qwen-4bit" in source
    assert "argparse.BooleanOptionalAction" in source
    assert "default=True" in source


def test_qwen_loader_uses_bitsandbytes_quantization():
    source = inspect.getsource(stt.QwenSemanticParser.load)

    assert "BitsAndBytesConfig" in source
    assert "load_in_4bit=True" in source
    assert 'bnb_4bit_quant_type="nf4"' in source
    assert "device_map=\"auto\"" in source
