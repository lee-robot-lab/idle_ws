import argparse
import contextlib
import io
import json
import logging
import os
import re
import sys
import tempfile
import termios
import time
import tty

sd = None
write = None
WhisperModel = None

fs = 16000
seconds = 5.0

WHISPER_INITIAL_PROMPT = (
    "로봇팔에게 내리는 한국어 음성 명령입니다. "
    "주요 단어는 빨간색, 초록색, 파란색, 박스, 블록, 바구니입니다. "
    "동작 단어는 집어, 잡아, 들어, 넣어, 담아, 놓아, 올려, 쌓아, 옮겨입니다. "
    "위치 단어는 왼쪽, 오른쪽, 앞쪽, 뒤쪽, 위에, 아래, 가까운, 가장 가까운, 제일 가까운, 먼, 가장 먼, 맨 왼쪽, 맨 오른쪽, 왼쪽 끝, 오른쪽 끝, 왼쪽 구석, 오른쪽 구석입니다. "
    "예시 문장: 빨간색 박스를 바구니에 넣어. "
    "예시 문장: 파란색 박스를 초록색 박스 위에 올려줘. "
    "예시 문장: 바구니에서 가장 가까운 블록을 집어줘. "
    "예시 문장: 바구니 왼쪽 블록을 바구니에 넣어. "
    "예시 문장: 바구니 왼쪽 블록을 빨간색 블록 위에 쌓아."
)

DEFAULT_QWEN_MODEL = "Qwen/Qwen2.5-3B-Instruct"

ALLOWED_ACTIONS = {"pick", "place", "pick_place", "stack"}
ALLOWED_OBJECTS = {
    "red_block",
    "blue_block",
    "green_block",
    "basket",
}
ALLOWED_QUERY_REFERENCES = ALLOWED_OBJECTS | {"robot"}
ALLOWED_QUERY_RELATIONS = {
    "nearest_to",
    "farthest_from",
    "left_of",
    "right_of",
    "front_of",
    "behind",
    "leftmost",
    "rightmost",
}
OBJECT_NAME_ALIASES = {
    "red_box": "red_block",
    "blue_box": "blue_block",
    "green_box": "green_block",
    "red_cube": "red_block",
    "blue_cube": "blue_block",
    "green_cube": "green_block",
}

QWEN_SYSTEM_PROMPT = """
당신은 한국어 로봇팔 명령을 실행 가능한 JSON 계획으로 변환하는 의미 파서다.
설명이나 마크다운 없이 JSON 객체 하나만 출력한다.

지원 동작:
- pick: 물체를 집는다.
- place: 이미 집고 있는 물체를 대상에 놓는다.
- pick_place: 물체를 집어서 바구니 같은 목적지로 옮긴다.
- stack: 한 블록을 다른 블록 위에 올린다.
사용자가 특정 물체를 "바구니에 넣어/담아/옮겨"라고 말하면 place가 아니라
반드시 pick_place다. place는 이미 로봇이 물체를 들고 있고 새 물체를
지정하지 않은 "바구니에 놓아" 같은 명령에만 사용한다.

지원 물체 이름:
- red_block, blue_block, green_block, basket
공간 관계의 기준점에는 위 물체 외에 robot을 사용할 수 있다.
robot은 object나 target이 될 수 없고 relation의 reference로만 사용한다.
사용자가 말하는 박스, 블록, 큐브는 모두 같은 물체다.
빨간 박스/빨간 블록/빨간 큐브는 항상 red_block으로 출력한다.
파란 박스/파란 블록/파란 큐브는 항상 blue_block으로 출력한다.
초록 박스/초록 블록/초록 큐브는 항상 green_block으로 출력한다.
red_box, blue_box, green_box 같은 이름은 출력하지 않는다.

색상으로 특정할 수 없는 물체는 object=null로 두고 object_query를 사용한다.
object_query 형식:
{
  "type": "block",
  "relations": [
    {"relation": "nearest_to", "reference": "basket"}
  ]
}
relation은 nearest_to, farthest_from, left_of, right_of, front_of, behind,
leftmost, rightmost 중 하나다. 여러 조건은 relations 배열에 모두 넣는다.
leftmost와 rightmost는 장면 전체에서 가장 왼쪽/오른쪽인 블록을 뜻하며
reference는 null이다. 그 외 relation은 reference가 필요하다.
"맨 왼쪽/오른쪽", "왼쪽/오른쪽 끝", "왼쪽/오른쪽 구석"도
각각 leftmost/rightmost로 분류한다.
놓을 대상도 직접 특정할 수 없으면 target=null로 두고 같은 형식의
target_query를 사용한다. "가장 오른쪽 블록 위에"는 rightmost_block 같은
이름을 만들지 말고 target_query의 rightmost relation으로 표현한다.
"로봇에서 가장 가까운/먼 박스"는 reference="robot"인
nearest_to/farthest_from relation으로 표현한다.
front_of와 behind의 기준 프레임은 아직 확정되지 않았으므로 입력 표현을
그대로 relation으로 분류하되 좌표 방향을 임의로 추론하지 않는다.

출력 형식:
{
  "success": true,
  "reason": "ok",
  "raw_text": "입력 문장",
  "needs_clarification": false,
  "clarification_question": null,
  "steps": [
    {
      "action": "pick",
      "object": "red_block",
      "object_query": null,
      "target": null,
      "target_query": null,
      "depends_on": []
    }
  ]
}

규칙:
1. 문장에 여러 행동이나 선행 조건이 있으면 steps를 실행 순서대로 분리한다.
2. depends_on에는 먼저 완료되어야 하는 step의 0부터 시작하는 인덱스를 넣는다.
3. 문맥상 생략된 목적어는 명확할 때만 복원한다.
   place는 로봇이 이미 집고 있는 물체를 놓는 동작이므로
   object=null, object_query=null을 허용하고 target 또는 target_query만 지정한다.
4. 대상이 여러 개라 결정할 수 없거나 정보가 부족하면 success=false,
   reason="ambiguous_command", needs_clarification=true로 설정하고
   clarification_question에 짧은 한국어 질문을 작성한다.
5. 상식으로 장면에 없는 물체나 관계를 만들어내지 않는다.
   rightmost_block, leftmost_block 같은 합성 물체 이름은 절대 reference로 만들지 않는다.
   가장 오른쪽/왼쪽 블록은 반드시 target_query 또는 object_query의
   rightmost/leftmost relation과 reference=null로만 표현한다.
6. RGB 탑다운 영상으로 판별하지 않는 on_top_of와 under 관계를 만들지 않는다.
7. 내부 추론 과정은 출력하지 않는다.

예시:
입력: 빨간 블록을 파란 블록 위에 올려줘
출력:
{"success":true,"reason":"ok","raw_text":"빨간 블록을 파란 블록 위에 올려줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"stack","object":"red_block","object_query":null,"target":"blue_block","target_query":null,"depends_on":[]}]}

입력: 파란 블록을 집으려면 위에 있는 빨간 블록부터 바구니로 치워줘
출력:
{"success":true,"reason":"ok","raw_text":"파란 블록을 집으려면 위에 있는 빨간 블록부터 바구니로 치워줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"pick_place","object":"red_block","object_query":null,"target":"basket","target_query":null,"depends_on":[]},{"action":"pick","object":"blue_block","object_query":null,"target":null,"target_query":null,"depends_on":[0]}]}

입력: 바구니 왼쪽에 있으면서 파란 블록과 가장 가까운 블록을 집어줘
출력:
{"success":true,"reason":"ok","raw_text":"바구니 왼쪽에 있으면서 파란 블록과 가장 가까운 블록을 집어줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"pick","object":null,"object_query":{"type":"block","relations":[{"relation":"left_of","reference":"basket"},{"relation":"nearest_to","reference":"blue_block"}]},"target":null,"target_query":null,"depends_on":[]}]}

입력: 가장 왼쪽 블록을 집어줘
출력:
{"success":true,"reason":"ok","raw_text":"가장 왼쪽 블록을 집어줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"pick","object":null,"object_query":{"type":"block","relations":[{"relation":"leftmost","reference":null}]},"target":null,"target_query":null,"depends_on":[]}]}

입력: 바구니 왼쪽 블록을 가장 오른쪽 블록 위에 올려줘
출력:
{"success":true,"reason":"ok","raw_text":"바구니 왼쪽 블록을 가장 오른쪽 블록 위에 올려줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"stack","object":null,"object_query":{"type":"block","relations":[{"relation":"left_of","reference":"basket"}]},"target":null,"target_query":{"type":"block","relations":[{"relation":"rightmost","reference":null}]},"depends_on":[]}]}

입력: 바구니에서 가장 가까운 박스를 바구니에서 가장 먼 박스 위에 올려줘
출력:
{"success":true,"reason":"ok","raw_text":"바구니에서 가장 가까운 박스를 바구니에서 가장 먼 박스 위에 올려줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"stack","object":null,"object_query":{"type":"block","relations":[{"relation":"nearest_to","reference":"basket"}]},"target":null,"target_query":{"type":"block","relations":[{"relation":"farthest_from","reference":"basket"}]},"depends_on":[]}]}

입력: 로봇에서 가장 가까운 박스를 집어줘
출력:
{"success":true,"reason":"ok","raw_text":"로봇에서 가장 가까운 박스를 집어줘","needs_clarification":false,"clarification_question":null,"steps":[{"action":"pick","object":null,"object_query":{"type":"block","relations":[{"relation":"nearest_to","reference":"robot"}]},"target":null,"target_query":null,"depends_on":[]}]}
""".strip()

correction_dict = {
    # object words
    "빡수": "박스", "박수": "박스", "박쓰": "박스",
    "박스스": "박스", "팩스": "박스", "백스": "박스",
    "큐브": "박스", "블럭": "박스", "블록": "박스", "빡스": "박스",

    # basket words often misrecognized by STT
    "강원": "바구니", "박원이": "바구니", "바군이": "바구니",
    "바구니에": "바구니에", "분이": "바구니", "방구니": "바구니",

    # color words
    "빨강색": "빨간색", "빨강": "빨간색",
    "파랑색": "파란색", "파랑": "파란색",
    "녹색": "초록색", "초록색": "초록색", "주름색": "초록색",

    # verb / phrase normalization
    "올려 줘": "올려줘",
    "올려놔": "올려놓아",
    "올려 놓아": "올려놓아",
    "쌓아 올려": "쌓아올려",
    "싸아": "쌓아",
    "싸": "쌓아",
    "너": "넣어",
    "느어": "넣어",
    "집어줘": "집어줘",
    "집어 줘": "집어줘",
    "누아": "놓아",
    "놔줘": "놓아줘",
}

color_map = {
    "빨간색": "red_block",
    "빨간": "red_block",
    "빨강": "red_block",
    "파란색": "blue_block",
    "파란": "blue_block",
    "파랑": "blue_block",
    "초록색": "green_block",
    "초록": "green_block",
    "녹색": "green_block",
}

STACK_WORDS = ["위에", "위로", "박스위", "올려", "쌓", "스택"]
BASKET_WORDS = ["바구니", "통", "상자"]
ROBOT_WORDS = ["로봇팔", "로봇", "기계팔"]
PUT_WORDS = ["넣", "담", "놓", "놔", "이동", "옮", "가져다", "넣어줘"]
PICK_WORDS = ["잡", "집", "집어", "들어", "들", "픽"]
PLACE_WORDS = ["놓", "놔", "내려"]

# 좌표/상태 추론이 필요한 표현들
NEAREST_WORDS = ["가장가까운", "제일가까운", "가까운", "근처", "근처에있는"]
FARTHEST_WORDS = ["가장먼", "제일먼", "먼", "가장멀리", "제일멀리", "멀리"]
LEFT_WORDS = ["왼쪽", "왼편", "좌측"]
RIGHT_WORDS = ["오른쪽", "오른편", "우측"]
FRONT_WORDS = ["앞쪽", "앞에", "앞"]
BACK_WORDS = ["뒤쪽", "뒤에", "뒤"]
LEFTMOST_WORDS = [
    "가장왼쪽", "제일왼쪽", "맨왼쪽",
    "왼쪽끝", "왼쪽구석", "왼쪽맨구석", "맨왼쪽구석",
]
RIGHTMOST_WORDS = [
    "가장오른쪽", "제일오른쪽", "맨오른쪽",
    "오른쪽끝", "오른쪽구석", "오른쪽맨구석", "맨오른쪽구석",
]


def correct_text(text):
    """STT 결과를 파서가 읽기 쉬운 형태로 정규화한다."""
    text = text.strip().replace(" ", "")

    for wrong, right in correction_dict.items():
        text = text.replace(wrong.replace(" ", ""), right.replace(" ", ""))

    return text


def _find_color_positions(text):
    """색상 단어를 텍스트 등장 순서대로 찾는다. 같은 색은 한 번만 유지한다."""
    matches = []

    for word, obj_name in color_map.items():
        start = 0
        while True:
            pos = text.find(word, start)
            if pos == -1:
                break
            matches.append((pos, len(word), obj_name, word))
            start = pos + len(word)

    # 같은 위치에서는 긴 단어 우선: '빨간색'이 '빨간'보다 먼저 처리됨
    matches.sort(key=lambda x: (x[0], -x[1]))

    unique_positions = []
    seen_obj = set()
    occupied = set()
    for pos, length, obj_name, word in matches:
        span = set(range(pos, pos + length))
        # '빨간색' 안의 '빨간'처럼 겹치는 짧은 단어 제거
        if occupied & span:
            continue
        occupied |= span
        if obj_name not in seen_obj:
            unique_positions.append((pos, length, obj_name, word))
            seen_obj.add(obj_name)

    return unique_positions


def find_colors(text):
    return [obj_name for _, _, obj_name, _ in _find_color_positions(text)]


def _contains_any(text, words):
    return any(word in text for word in words)


def _first_connector_between(text, left_pos, right_pos):
    connectors = ["박스위에", "박스위", "위에", "위로", "위"]
    for conn in connectors:
        cpos = text.find(conn)
        if cpos != -1 and left_pos < cpos < right_pos:
            return conn
    return None


def _find_first_pos(text, words):
    """words 중 text에 가장 먼저 나오는 단어 위치를 반환한다."""
    positions = []
    for word in words:
        pos = text.find(word)
        if pos != -1:
            positions.append((pos, word))
    if not positions:
        return None, None
    return min(positions, key=lambda x: x[0])


def _find_reference_positions(text):
    """바구니/색상 블록처럼 기준점으로 쓸 수 있는 대상의 위치를 찾는다."""
    refs = []

    for basket_word in BASKET_WORDS:
        start = 0
        while True:
            pos = text.find(basket_word, start)
            if pos == -1:
                break
            refs.append((pos, "basket", basket_word))
            start = pos + len(basket_word)

    for robot_word in ROBOT_WORDS:
        start = 0
        while True:
            pos = text.find(robot_word, start)
            if pos == -1:
                break
            refs.append((pos, "robot", robot_word))
            start = pos + len(robot_word)

    for pos, length, obj_name, word in _find_color_positions(text):
        refs.append((pos, obj_name, word))

    refs.sort(key=lambda x: x[0])
    return refs


def _nearest_reference_before(text, keyword_pos):
    refs = _find_reference_positions(text)
    before = [r for r in refs if r[0] < keyword_pos]
    if before:
        return before[-1][1]
    return None


def _nearest_reference_after(text, keyword_pos):
    refs = _find_reference_positions(text)
    after = [r for r in refs if r[0] > keyword_pos]
    if after:
        return after[0][1]
    return None


def _make_object_query(selector, reference, object_type="block"):
    return {
        "type": object_type,
        "selector": selector,
        "reference": reference,
    }


def _detect_object_query(text):
    """
    '바구니에서 가장 가까운 박스', '바구니 왼쪽 블록'처럼
    비전 좌표와 planner 계산이 필요한 object query를 뽑는다.
    """
    # 1) 장면 전체 서열: 가장 왼쪽/오른쪽 블록
    if _contains_any(text, LEFTMOST_WORDS) and ("박스" in text or "블록" in text):
        return _make_object_query("leftmost", None)

    if _contains_any(text, RIGHTMOST_WORDS) and ("박스" in text or "블록" in text):
        return _make_object_query("rightmost", None)

    # 2) 거리 기반: 바구니에서 가장 가까운/먼 박스
    nearest_pos, _ = _find_first_pos(text, NEAREST_WORDS)
    if nearest_pos is not None and ("박스" in text or "블록" in text):
        ref = _nearest_reference_before(text, nearest_pos) or _nearest_reference_after(text, nearest_pos)
        if ref:
            return _make_object_query("nearest_to", ref)

    farthest_pos, _ = _find_first_pos(text, FARTHEST_WORDS)
    if farthest_pos is not None and ("박스" in text or "블록" in text):
        ref = _nearest_reference_before(text, farthest_pos) or _nearest_reference_after(text, farthest_pos)
        if ref:
            return _make_object_query("farthest_from", ref)

    # 3) 방향 기반: 바구니 왼쪽/오른쪽/앞/뒤 블록
    direction_sets = [
        ("left_of", LEFT_WORDS),
        ("right_of", RIGHT_WORDS),
        ("front_of", FRONT_WORDS),
        ("behind", BACK_WORDS),
    ]
    for selector, words in direction_sets:
        dir_pos, _ = _find_first_pos(text, words)
        if dir_pos is None:
            continue
        if not ("박스" in text or "블록" in text):
            continue
        ref = _nearest_reference_before(text, dir_pos) or _nearest_reference_after(text, dir_pos)
        if ref:
            return _make_object_query(selector, ref)

    return None


def _split_stack_clauses(text):
    """'<집을 물체>를 <놓을 물체> 위에' 문장을 두 물체 절로 분리한다."""
    connector_positions = [
        pos
        for connector in ("위에", "위로")
        if (pos := text.find(connector)) != -1
    ]
    if not connector_positions:
        return None, None

    connector_pos = min(connector_positions)
    before_connector = text[:connector_pos]
    separator_positions = [
        pos
        for particle in ("을", "를")
        if (pos := before_connector.rfind(particle)) != -1
    ]
    if not separator_positions:
        return None, None

    separator_pos = max(separator_positions)
    object_clause = before_connector[:separator_pos]
    target_clause = before_connector[separator_pos + 1:]
    if not object_clause or not target_clause:
        return None, None
    return object_clause, target_clause


def _has_basket_as_put_target(text):
    """바구니가 기준점이 아니라 '넣을 대상'으로 쓰였는지 판단한다.

    주의: "바구니에서" 안에도 "바구니에"가 부분 문자열로 들어가므로
    단순히 "바구니에" 포함 여부만 보면 오판한다.
    "바구니에넣어", "바구니안에"처럼 실제 목적지 표현만 잡는다.
    """
    target_phrases = [
        "바구니에넣", "바구니에담", "바구니에놓", "바구니에놔",
        "바구니안에", "바구니속에",
        "통에넣", "통에담", "상자에넣", "상자에담"
    ]
    if any(p in text for p in target_phrases):
        return True

    # 목적지가 문장 앞에 오는 어순:
    # "바구니에 빨간색 박스를 넣어줘"
    destination_first = re.search(
        r"(?:바구니|통|상자)(?:에|안에|속에).{0,30}?(?:넣|담|놓|놔)",
        text,
    )
    return destination_first is not None


def _select_stack_target(text, unique_positions, object_query=None):
    """
    stack 문장에서 target block을 찾는다.
    - object_query가 있으면 색상 블록은 보통 target으로 본다.
      예: '바구니 왼쪽 블록을 빨간색 블록 위에 쌓아'
    - 색상 두 개면 기존 규칙을 유지한다.
    """
    colors = [obj_name for _, _, obj_name, _ in unique_positions]

    if object_query and colors:
        return colors[0]

    if len(colors) < 2:
        return None

    obj = colors[0]
    target = colors[1]

    # 반대형: "파란색 위에 빨간색 올려" -> 첫 색이 target, 두 번째 색이 object
    pos1 = unique_positions[0][0]
    pos2 = unique_positions[1][0]
    if _first_connector_between(text, pos1, pos2):
        obj = colors[1]
        target = colors[0]

    return target, obj


def parse_command(text):
    """
    한국어 음성 명령을 로봇 명령 JSON으로 변환한다.

    직접 지정 예:
    - '빨간색을 파란색 위에 올려'      -> object=red, target=blue
    - '파란색 위에 빨간색 올려'        -> object=red, target=blue
    - '빨간색을 바구니에 넣어'          -> object=red, target=basket
    - '빨간색 집어'                    -> action=pick, target=None

    추론 필요 예:
    - '바구니에서 가장 가까운 박스를 집어'
      -> object_query={selector: nearest_to, reference: basket}
    - '바구니 왼쪽 블록을 바구니에 넣어'
      -> object_query={selector: left_of, reference: basket}, target=basket
    - '바구니 왼쪽 블록을 빨간색 블록 위에 쌓아'
      -> object_query={selector: left_of, reference: basket}, target=red_block
    """
    text = correct_text(text)
    unique_positions = _find_color_positions(text)
    colors = [obj_name for _, _, obj_name, _ in unique_positions]

    object_clause, target_clause = _split_stack_clauses(text)
    object_query = _detect_object_query(object_clause or text)
    target_query = _detect_object_query(target_clause) if target_clause else None

    action = "unknown"
    obj = None if object_query else (colors[0] if colors else None)
    target = None

    has_stack = _contains_any(text, STACK_WORDS)
    has_put = _contains_any(text, PUT_WORDS)
    has_pick = _contains_any(text, PICK_WORDS)
    has_place = _contains_any(text, PLACE_WORDS)
    basket_is_target = _has_basket_as_put_target(text)

    # 1) target/object 결정
    if has_stack:
        if target_query is None:
            stack_target = _select_stack_target(text, unique_positions, object_query=object_query)
            if isinstance(stack_target, tuple):
                target, obj = stack_target
            else:
                target = stack_target
    elif basket_is_target:
        target = "basket"

    # 2) action 결정
    # '바구니 왼쪽 블록을 바구니에 넣어'는 pick_place
    if basket_is_target and (has_put or has_place or has_pick or has_stack):
        action = "pick_place"
    elif has_stack:
        action = "stack"
    elif has_put and target == "basket":
        action = "pick_place"
    elif has_pick:
        action = "pick"
    elif has_place:
        action = "place"

    # 3) 성공/실패 판단
    success = True
    reason = "ok"
    has_object_reference = obj is not None or object_query is not None

    if action == "unknown":
        success = False
        reason = "unknown_action"
    elif not has_object_reference:
        success = False
        reason = "unknown_object"
    elif action in ["pick_place", "stack", "place"] and target is None and target_query is None:
        success = False
        reason = "unknown_target"

    return {
        "success": success,
        "reason": reason,
        "raw_text": text,
        "action": action,
        "object": obj,
        "object_query": object_query,
        "target": target,
        "target_query": target_query,
    }


def _legacy_query_to_semantic_query(object_query):
    if object_query is None:
        return None

    selector = object_query.get("selector")
    reference = object_query.get("reference")
    if selector is None:
        return None

    return {
        "type": object_query.get("type", "block"),
        "relations": [
            {
                "relation": selector,
                "reference": reference,
            }
        ],
    }


def rule_result_to_plan(command):
    """기존 규칙 파서 결과를 Qwen 계획과 동일한 형태로 변환한다."""
    success = bool(command.get("success"))
    steps = []

    if success:
        steps.append(
            {
                "action": command.get("action"),
                "object": command.get("object"),
                "object_query": _legacy_query_to_semantic_query(
                    command.get("object_query")
                ),
                "target": command.get("target"),
                "target_query": _legacy_query_to_semantic_query(
                    command.get("target_query")
                ),
                "depends_on": [],
            }
        )

    return {
        "success": success,
        "reason": command.get("reason", "unknown"),
        "raw_text": command.get("raw_text", ""),
        "needs_clarification": not success,
        "clarification_question": None,
        "steps": steps,
        "parser": "rule",
    }


def _extract_json_object(text):
    """모델 출력에서 첫 번째 완전한 JSON 객체를 추출한다."""
    text = text.strip()
    fenced = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if fenced:
        text = fenced.group(1)

    start = text.find("{")
    if start == -1:
        raise ValueError("Qwen 출력에 JSON 객체가 없습니다.")

    depth = 0
    in_string = False
    escaped = False

    for index in range(start, len(text)):
        char = text[index]

        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]

    raise ValueError("Qwen 출력의 JSON 객체가 닫히지 않았습니다.")


def _validate_object_query(query, field_name):
    if query is None:
        return
    if not isinstance(query, dict):
        raise ValueError(f"{field_name}는 객체 또는 null이어야 합니다.")
    if query.get("type") != "block":
        raise ValueError(f"{field_name}.type은 block이어야 합니다.")

    relations = query.get("relations")
    if not isinstance(relations, list) or not relations:
        raise ValueError(f"{field_name}.relations는 비어 있지 않은 배열이어야 합니다.")

    for relation in relations:
        if not isinstance(relation, dict):
            raise ValueError(f"{field_name}.relations 항목은 객체여야 합니다.")
        relation_name = relation.get("relation")
        reference = relation.get("reference")
        if relation_name not in ALLOWED_QUERY_RELATIONS:
            raise ValueError(f"허용되지 않은 관계입니다: {relation_name}")
        if relation_name in {"leftmost", "rightmost"}:
            if reference is not None:
                raise ValueError(f"{relation_name}의 reference는 null이어야 합니다.")
        elif reference not in ALLOWED_QUERY_REFERENCES:
            raise ValueError(f"허용되지 않은 기준 물체입니다: {reference}")


def normalize_llm_plan(plan):
    """LLM 물체 별칭을 canonical 이름으로 바꾸고 중복 합성 관계를 제거한다."""
    if not isinstance(plan, dict):
        return plan

    for step in plan.get("steps", []):
        if not isinstance(step, dict):
            continue

        step["object"] = OBJECT_NAME_ALIASES.get(
            step.get("object"),
            step.get("object"),
        )
        step["target"] = OBJECT_NAME_ALIASES.get(
            step.get("target"),
            step.get("target"),
        )

        # 명시적 물체를 목적지로 옮기는 명령은 집기까지 포함하므로 pick_place다.
        if (
            step.get("action") == "place"
            and (step.get("object") is not None or step.get("object_query") is not None)
            and (step.get("target") is not None or step.get("target_query") is not None)
        ):
            step["action"] = "pick_place"

        for query_name in ("object_query", "target_query"):
            query = step.get(query_name)
            if not isinstance(query, dict):
                continue
            for relation in query.get("relations", []):
                if not isinstance(relation, dict):
                    continue
                reference = relation.get("reference")
                relation["reference"] = OBJECT_NAME_ALIASES.get(
                    reference,
                    reference,
                )

        target_query = step.get("target_query")
        target_selectors = set()
        if isinstance(target_query, dict):
            for relation in target_query.get("relations", []):
                if isinstance(relation, dict):
                    selector = relation.get("relation")
                    if selector in {"leftmost", "rightmost"}:
                        target_selectors.add(selector)

        object_query = step.get("object_query")
        if not target_selectors or not isinstance(object_query, dict):
            continue

        relations = object_query.get("relations")
        if not isinstance(relations, list):
            continue

        normalized_relations = []
        for relation in relations:
            if not isinstance(relation, dict):
                normalized_relations.append(relation)
                continue

            reference = relation.get("reference")
            is_redundant_synthetic_reference = any(
                reference == f"{selector}_block"
                for selector in target_selectors
            )
            if is_redundant_synthetic_reference:
                continue
            normalized_relations.append(relation)

        # 유효한 출발 물체 조건이 남을 때만 중복 항목 제거를 적용한다.
        if normalized_relations:
            object_query["relations"] = normalized_relations

    return plan


def align_direct_rule_objects(plan, corrected_text):
    """
    Qwen이 명시 색상 블록을 관계 query로 과해석한 경우만 교정한다.
    예: "파란색 박스를 초록색 박스 위에"는 blue_block -> green_block이다.
    """
    if not isinstance(plan, dict) or not plan.get("success"):
        return plan

    steps = plan.get("steps")
    if not isinstance(steps, list) or len(steps) != 1:
        return plan

    rule_plan = rule_result_to_plan(parse_command(corrected_text))
    rule_steps = rule_plan.get("steps") or []
    if not rule_plan.get("success") or len(rule_steps) != 1:
        return plan

    step = steps[0]
    rule_step = rule_steps[0]
    if not isinstance(step, dict) or step.get("action") != rule_step.get("action"):
        return plan

    for value_name, query_name in (
        ("object", "object_query"),
        ("target", "target_query"),
    ):
        rule_value = rule_step.get(value_name)
        if rule_value is None or rule_step.get(query_name) is not None:
            continue
        if step.get(value_name) is None and step.get(query_name) is not None:
            step[value_name] = rule_value
            step[query_name] = None

    return plan


def validate_semantic_plan(plan, raw_text):
    """LLM 출력을 허용된 로봇 명령 스키마로 제한한다."""
    if not isinstance(plan, dict):
        raise ValueError("명령 계획은 JSON 객체여야 합니다.")

    success = plan.get("success")
    if not isinstance(success, bool):
        raise ValueError("success는 boolean이어야 합니다.")

    needs_clarification = plan.get("needs_clarification", False)
    if not isinstance(needs_clarification, bool):
        raise ValueError("needs_clarification은 boolean이어야 합니다.")

    steps = plan.get("steps")
    if not isinstance(steps, list):
        raise ValueError("steps는 배열이어야 합니다.")
    if success and not steps:
        raise ValueError("성공한 명령에는 한 개 이상의 step이 필요합니다.")

    normalized_steps = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            raise ValueError(f"steps[{index}]는 객체여야 합니다.")

        action = step.get("action")
        obj = step.get("object")
        object_query = step.get("object_query")
        target = step.get("target")
        target_query = step.get("target_query")
        depends_on = step.get("depends_on", [])

        if action not in ALLOWED_ACTIONS:
            raise ValueError(f"허용되지 않은 action입니다: {action}")
        if obj is not None and obj not in ALLOWED_OBJECTS - {"basket"}:
            raise ValueError(f"허용되지 않은 object입니다: {obj}")
        if action != "place" and obj is None and object_query is None:
            raise ValueError(f"steps[{index}]에 object 또는 object_query가 필요합니다.")
        if obj is not None and object_query is not None:
            raise ValueError(f"steps[{index}]는 object와 object_query를 동시에 가질 수 없습니다.")
        if action == "place" and (obj is not None or object_query is not None):
            raise ValueError("place action은 이미 집은 물체를 사용하므로 object/object_query가 null이어야 합니다.")
        if target is not None and target not in ALLOWED_OBJECTS:
            raise ValueError(f"허용되지 않은 target입니다: {target}")
        if target is not None and target_query is not None:
            raise ValueError(f"steps[{index}]는 target과 target_query를 동시에 가질 수 없습니다.")

        _validate_object_query(object_query, f"steps[{index}].object_query")
        _validate_object_query(target_query, f"steps[{index}].target_query")

        if action == "pick" and (target is not None or target_query is not None):
            raise ValueError("pick action은 target/target_query를 가질 수 없습니다.")
        if action in {"pick_place", "stack", "place"} and target is None and target_query is None:
            raise ValueError(f"{action} action에는 target 또는 target_query가 필요합니다.")
        if action == "stack" and target == "basket":
            raise ValueError("stack target은 basket일 수 없습니다.")

        if not isinstance(depends_on, list):
            raise ValueError("depends_on은 배열이어야 합니다.")
        if any(
            not isinstance(dependency, int)
            or dependency < 0
            or dependency >= index
            for dependency in depends_on
        ):
            raise ValueError(
                f"steps[{index}].depends_on은 앞선 step 인덱스만 참조할 수 있습니다."
            )

        normalized_steps.append(
            {
                "action": action,
                "object": obj,
                "object_query": object_query,
                "target": target,
                "target_query": target_query,
                "depends_on": sorted(set(depends_on)),
            }
        )

    clarification_question = plan.get("clarification_question")
    if needs_clarification and not clarification_question:
        raise ValueError("재질문이 필요하면 clarification_question이 있어야 합니다.")

    return {
        "success": success,
        "reason": str(plan.get("reason", "ok" if success else "unknown")),
        "raw_text": raw_text,
        "needs_clarification": needs_clarification,
        "clarification_question": clarification_question,
        "steps": normalized_steps,
        "parser": "qwen",
    }


class QwenSemanticParser:
    """Qwen2.5-Instruct를 이용해 한국어 명령을 구조화한다."""

    def __init__(
        self,
        model_name=DEFAULT_QWEN_MODEL,
        max_new_tokens=512,
        use_4bit=True,
    ):
        self.model_name = model_name
        self.max_new_tokens = max_new_tokens
        self.use_4bit = use_4bit
        self.model = None
        self.tokenizer = None
        self.torch = None
        self.device = None

    def load(self):
        if self.model is not None:
            return

        try:
            import torch
            import huggingface_hub
            from transformers import (
                AutoModelForCausalLM,
                AutoTokenizer,
                BitsAndBytesConfig,
            )
        except Exception as exc:
            raise RuntimeError(
                "Qwen 실행에는 torch, transformers, bitsandbytes가 필요합니다."
            ) from exc

        logging.getLogger("huggingface_hub").setLevel(logging.ERROR)
        huggingface_hub.logging.set_verbosity_error()
        self.torch = torch
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.use_4bit and self.device != "cuda":
            raise RuntimeError("4-bit Qwen 실행에는 CUDA GPU가 필요합니다.")

        self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
        if self.use_4bit:
            quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=True,
                bnb_4bit_compute_dtype=torch.float16,
            )
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                quantization_config=quantization_config,
                device_map="auto",
            )
        else:
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype="auto",
            )
            self.model.to(self.device)
        self.model.eval()

    def parse(self, raw_text, corrected_text=None):
        self.load()
        corrected_text = corrected_text or correct_text(raw_text)

        user_content = (
            f"Whisper 원문: {raw_text.strip()}\n"
            f"정규화 문장: {corrected_text}\n"
            "위 명령을 지정된 JSON 형식으로 변환하라."
        )
        messages = [
            {"role": "system", "content": QWEN_SYSTEM_PROMPT},
            {"role": "user", "content": user_content},
        ]
        prompt = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True,
        )
        inputs = self.tokenizer([prompt], return_tensors="pt").to(self.device)

        with self.torch.inference_mode():
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=self.max_new_tokens,
                do_sample=False,
                repetition_penalty=1.05,
                pad_token_id=self.tokenizer.eos_token_id,
            )

        generated_ids = generated_ids[:, inputs.input_ids.shape[1]:]
        response = self.tokenizer.batch_decode(
            generated_ids,
            skip_special_tokens=True,
        )[0]
        json_text = _extract_json_object(response)
        plan = json.loads(json_text)
        plan = normalize_llm_plan(plan)
        plan = align_direct_rule_objects(plan, corrected_text)
        return validate_semantic_plan(plan, raw_text.strip())


def parse_with_mode(raw_text, parser_mode, qwen_parser=None):
    corrected_text = correct_text(raw_text)

    if parser_mode == "rule":
        return rule_result_to_plan(parse_command(corrected_text))

    if qwen_parser is None:
        raise ValueError("qwen 또는 hybrid 모드에는 QwenSemanticParser가 필요합니다.")

    try:
        return qwen_parser.parse(raw_text, corrected_text)
    except Exception as exc:
        if parser_mode == "qwen":
            raise

        fallback = rule_result_to_plan(parse_command(corrected_text))
        fallback["parser"] = "rule_fallback"
        fallback["llm_error"] = str(exc)
        return fallback


def transcribe_audio_file(model, wav_path):
    """faster-whisper로 wav 파일을 텍스트로 변환한다."""
    segments, info = model.transcribe(
        wav_path,
        language="ko",
        beam_size=5,
        initial_prompt=WHISPER_INITIAL_PROMPT,
        condition_on_previous_text=False,
        vad_filter=True,
    )

    result_text = ""
    for segment in segments:
        result_text += segment.text
    return result_text


def load_voice_dependencies():
    """텍스트 파싱 모드에서는 무거운 음성 라이브러리를 import하지 않는다."""
    global sd, write, WhisperModel

    try:
        import sounddevice as sounddevice
        from scipy.io.wavfile import write as wav_write
        from faster_whisper import WhisperModel as FasterWhisperModel
    except Exception as exc:
        raise RuntimeError(
            "음성 모드에는 sounddevice, scipy, faster-whisper가 필요합니다."
        ) from exc

    sd = sounddevice
    write = wav_write
    WhisperModel = FasterWhisperModel


def _read_terminal_key():
    """cbreak 모드의 터미널에서 Enter 없이 한 글자를 읽는다."""
    return sys.stdin.read(1)


def _record_until_space():
    """스페이스바를 다시 누를 때까지 마이크 프레임을 수집한다."""
    import numpy as np

    frames = []
    started_at = time.monotonic()
    quit_requested = False

    def audio_callback(indata, frame_count, time_info, status):
        if status:
            print(f"\n오디오 상태: {status}", file=sys.stderr)
        frames.append(indata.copy())

    print("녹음 중... 스페이스바를 다시 누르면 종료합니다. [q] 종료")

    with sd.InputStream(
        samplerate=fs,
        channels=1,
        dtype="float32",
        callback=audio_callback,
    ):
        while True:
            key = _read_terminal_key()

            if key.lower() == "q":
                quit_requested = True
                break

            # 첫 스페이스를 길게 눌렀을 때 발생하는 키 자동 반복을 무시한다.
            if key == " " and time.monotonic() - started_at >= 0.35:
                break

    if not frames:
        return None, quit_requested

    return np.concatenate(frames, axis=0), quit_requested


def print_fail_message(command):
    if command.get("needs_clarification"):
        question = command.get("clarification_question")
        if question:
            print(question)
            return

    reason = command.get("reason", "unknown")

    if reason == "unknown_action":
        if command.get("object") is not None or command.get("object_query") is not None:
            print("물체만 감지되었습니다. 예: '초록색 박스를 바구니에 넣어'처럼 동작과 대상을 함께 말해주세요.")
        else:
            print("동작을 인식하지 못했습니다. 예: '박스를 올려'처럼 전체 문장으로 다시 말해주세요.")
    elif reason == "unknown_object":
        print("물체를 인식하지 못했습니다. 예: '초록색 박스' 또는 '바구니 왼쪽 블록'처럼 물체를 포함해 말해주세요.")
    elif reason == "unknown_target":
        print("대상을 인식하지 못했습니다. 예: '빨간색 박스 위에' 또는 '바구니에'처럼 대상 위치를 포함해 말해주세요.")
    else:
        print("명령을 이해하지 못했습니다. 문장을 다시 말해주세요.")


ACTION_LABELS = {
    "pick": "Pick",
    "place": "Place",
    "pick_place": "Pick & Place",
    "stack": "Stack",
}

OBJECT_LABELS = {
    "red_block": "red block",
    "blue_block": "blue block",
    "green_block": "green block",
    "basket": "basket",
    "robot": "robot",
}

RELATION_LABELS = {
    "nearest_to": "nearest block to {reference}",
    "farthest_from": "farthest block from {reference}",
    "left_of": "block left of {reference}",
    "right_of": "block right of {reference}",
    "front_of": "block in front of {reference}",
    "behind": "block behind {reference}",
    "leftmost": "leftmost block",
    "rightmost": "rightmost block",
}


def _display_object_name(name):
    if name is None:
        return "none"
    return OBJECT_LABELS.get(name, name)


def _display_query(query):
    if not isinstance(query, dict):
        return None

    relations = query.get("relations") or []
    labels = []
    for relation in relations:
        if not isinstance(relation, dict):
            continue
        relation_name = relation.get("relation")
        template = RELATION_LABELS.get(relation_name)
        if template is None:
            labels.append(str(relation_name))
            continue
        reference = _display_object_name(relation.get("reference"))
        labels.append(template.format(reference=reference))

    if not labels:
        return "조건 블록"
    return " + ".join(labels)


def _display_step_value(value, query):
    query_label = _display_query(query)
    if query_label is not None:
        return query_label
    return _display_object_name(value)


def print_command_summary(command):
    print("Command:")

    steps = command.get("steps") or []
    if not steps:
        print(f"  Failed: {command.get('reason', 'unknown')}")
        return

    for index, step in enumerate(steps):
        action = ACTION_LABELS.get(step.get("action"), step.get("action"))
        obj = _display_step_value(step.get("object"), step.get("object_query"))
        target = _display_step_value(step.get("target"), step.get("target_query"))

        if index > 0:
            print()
        print(f"  Action : {action}")
        if step.get("action") != "place":
            print(f"  Object : {obj}")
        if step.get("target") is not None or step.get("target_query") is not None:
            print(f"  Target : {target}")
        if step.get("depends_on"):
            dependencies = ", ".join(str(dep + 1) for dep in step["depends_on"])
            print(f"  Depends: {dependencies}")


def run_test_mode(parser_mode, qwen_parser=None):
    test_sentences = [
        "빨간색 박스를 바구니에 넣어",
        "파란색 박스를 초록색 박스 위에 올려줘",
        "초록색 위에 빨간색 올려줘",
        "바구니에서 가장 가까운 블록을 집어줘",
        "바구니에서 가장 가까운 박스를 바구니에 넣어",
        "바구니에서 가장 먼 블록을 파란색 블록 위에 쌓아",
        "바구니에서 가장 가까운 박스를 바구니에서 가장 먼 박스 위에 올려줘",
        "로봇에서 가장 가까운 박스를 집어줘",
        "로봇에서 가장 먼 박스를 로봇에서 가장 가까운 박스 위에 올려줘",
        "바구니 왼쪽 블록을 바구니에 넣어",
        "바구니 오른쪽 블록을 초록색 블록 위에 쌓아",
        "바구니 왼쪽 블록을 가장 오른쪽 블록 위에 올려줘",
        "바구니 앞쪽 블록을 빨간색 블록 위에 올려줘",
        "바구니 뒤쪽 블록을 집어줘",
    ]

    for sentence in test_sentences:
        command = parse_with_mode(sentence, parser_mode, qwen_parser)
        print("\n입력:", sentence)
        print("명령:", json.dumps(command, ensure_ascii=False, indent=2))


def run_text_mode(text, parser_mode, qwen_parser=None):
    command = parse_with_mode(text, parser_mode, qwen_parser)
    print(json.dumps(command, ensure_ascii=False, indent=2))


def run_voice_mode(parser_mode, qwen_parser=None):
    try:
        load_voice_dependencies()
    except RuntimeError as exc:
        print(exc)
        return

    model = None
    try:
        model = WhisperModel("small", device="cpu", compute_type="int8")
    except Exception as e:
        print("모델 초기화 실패:", e)
        return

    if not sys.stdin.isatty():
        print("스페이스바 입력 모드는 대화형 터미널에서 실행해야 합니다.")
        return

    if qwen_parser is not None:
        try:
            # Hugging Face 경고와 모델 weight 진행률은 사용자 UI에서 숨긴다.
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                qwen_parser.load()
        except Exception as exc:
            if parser_mode == "qwen":
                print("Qwen 모델 초기화 실패:", exc)
                return
            print("Qwen 모델 초기화 실패. 규칙 파서 fallback을 사용합니다:", exc)

    stdin_fd = sys.stdin.fileno()
    original_terminal_settings = termios.tcgetattr(stdin_fd)

    try:
        tty.setcbreak(stdin_fd)

        while True:
            print("\n대기 중...")
            key = _read_terminal_key()

            if key.lower() == "q":
                print("종료")
                break

            if key != " ":
                continue

            audio, quit_requested = _record_until_space()
            if quit_requested:
                print("\n종료")
                break
            if audio is None or len(audio) < int(fs * 0.2):
                print("\n녹음이 너무 짧습니다. 다시 시도하세요.")
                continue

            duration = len(audio) / fs
            print(f"\n녹음 종료 ({duration:.1f}초). 음성 인식 중...")

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                temp_path = tmp.name

            try:
                write(temp_path, fs, audio)

                result_text = transcribe_audio_file(model, temp_path)
                if not result_text.strip():
                    print("음성을 인식하지 못했습니다. 다시 시도하세요.")
                    continue

                corrected = correct_text(result_text)
                command = parse_with_mode(result_text, parser_mode, qwen_parser)

                print("인식:", corrected)
                print_command_summary(command)

                if not command.get("success"):
                    print_fail_message(command)
                    continue

                print("명령 인식 완료. 다음 명령을 입력할 수 있습니다.")

            finally:
                if os.path.exists(temp_path):
                    os.remove(temp_path)

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        termios.tcsetattr(
            stdin_fd,
            termios.TCSADRAIN,
            original_terminal_settings,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="음성 입력 없이 파서 테스트 문장을 실행한다.")
    parser.add_argument("--text", help="음성 입력 없이 한 문장을 파싱한다.")
    parser.add_argument(
        "--parser",
        choices=["rule", "qwen", "hybrid"],
        default="hybrid",
        help="명령 파서 선택. hybrid는 Qwen 실패 시 규칙 파서를 사용한다.",
    )
    parser.add_argument(
        "--qwen-model",
        default=DEFAULT_QWEN_MODEL,
        help="사용할 Hugging Face Qwen2.5 Instruct 모델 이름 또는 로컬 경로.",
    )
    parser.add_argument(
        "--qwen-4bit",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Qwen을 bitsandbytes 4-bit 양자화로 로드한다. 기본값: true.",
    )
    args = parser.parse_args()

    qwen_parser = None
    if args.parser in {"qwen", "hybrid"}:
        qwen_parser = QwenSemanticParser(
            args.qwen_model,
            use_4bit=args.qwen_4bit,
        )

    if args.test:
        run_test_mode(args.parser, qwen_parser)
    elif args.text:
        run_text_mode(args.text, args.parser, qwen_parser)
    else:
        run_voice_mode(args.parser, qwen_parser)


if __name__ == "__main__":
    main()
