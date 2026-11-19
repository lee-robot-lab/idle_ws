# ================================================================
# conftest.py
# 설명: pytest가 src/ml을 import 루트로 인식하도록 sys.path에 추가.
# 사용법: src/ml/ 에서 `python -m pytest tests/ -v`
# ================================================================
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
