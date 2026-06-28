# [Archive] 에이전트 인수인계 스냅샷 보관함

## 이 폴더의 용도

에이전트 세션이 끝날 때 당시 작업 상태를 **날짜_주제.md** 형식으로 스냅샷으로 저장하는 곳이다.

- 파일명: `YYYY-MM-DD_주제.md`
- 내용: 직전 세션에서 어디까지 했는지, 다음 에이전트가 이어받을 때 필요한 컨텍스트
- 최신 canonical 인계 문서는 여기가 아니라 [`docs/agent/pick_place_tuning_handoff.md`](../pick_place_tuning_handoff.md) 에 유지한다

## 에이전트 인계 프로토콜

1. 세션 시작 시 `docs/agent/pick_place_tuning_handoff.md` 를 먼저 읽는다
2. 작업 완료 후 canonical 인계 문서를 갱신한다
3. 이전 버전을 이 폴더에 날짜 파일로 보관한다
4. archive 파일은 수정하지 않는다 — 히스토리 참조용
