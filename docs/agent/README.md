# Agent Handoff Rules

`docs/agent`는 에이전트 간 작업 인수인계를 위한 문서 영역이다.

## Canonical handoff

- 최신 실행 기준은 항상 `docs/agent/pick_place_tuning_handoff.md` 하나다.
- 다음 작업자는 먼저 canonical handoff를 읽고, 필요한 경우 archive와 timeline을 확인한다.
- canonical handoff 첫 화면에는 실행 커맨드, 현재 결론, 다음 작업이 빠르게 보여야 한다.
- 긴 실험 로그와 과거 판단 근거는 archive나 하단 notes로 보낸다.

## Archive

- `docs/agent/archive/claude/`는 Claude가 남긴 원본 handoff snapshot을 보관한다.
- `docs/agent/archive/codex/`는 Codex가 남긴 원본 handoff snapshot을 보관한다.
- archive 파일은 서로 섞어 고치지 않는다.
- 기존 archive 내용을 수정해야 할 때는 원본을 덮어쓰지 말고 새 날짜 snapshot을 추가한다.

## Timeline

- 에이전트 간 작업 흐름은 `docs/agent/timeline.md`에 시간순으로 기록한다.
- 각 entry에는 `Agent`, `Based on`, `Changed`, `Evidence`, `Next`를 적는다.
- timeline은 "내가 마지막으로 한 일"과 "그 사이 상대가 한 일"을 빠르게 확인하기 위한 인덱스다.

## Reproducibility

- 권장 launch command는 실제 launch file의 exposed parameter와 일치해야 한다.
- 새 파라미터를 handoff에 쓰기 전에 launch argument와 Node parameter 전달 여부를 확인한다.
- 실험 결론에는 가능하면 CSV 파일명, 분석 명령, 핵심 수치를 붙인다.
- 금지된 접근은 이유와 실패 증거를 함께 적는다.
