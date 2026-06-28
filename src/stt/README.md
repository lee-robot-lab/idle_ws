# Lightweight STT Qwen Parser

This is a standalone lightweight copy of the STT command parser.

The Qwen parser loads `Qwen/Qwen2.5-3B-Instruct` with bitsandbytes 4-bit
quantization by default, so it should use much less VRAM than the original
FP16/BF16 load path.

## Install

```bash
cd /home/parkshinyoung/stt
python3 -m pip install -r requirements.txt
```

## Run

```bash
python3 stt.py --parser qwen --text "빨간 블록을 바구니에 넣어줘"
```

The default is 4-bit quantized Qwen. To run the old full-precision path:

```bash
python3 stt.py --parser qwen --no-qwen-4bit --text "빨간 블록을 바구니에 넣어줘"
```

## Check VRAM

```bash
watch -n 1 nvidia-smi
```

## Test

```bash
python3 -m pytest -q
```
