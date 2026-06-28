# MuJoCo Phase RL Runtime Outputs

## 남긴 파일

- `final/vision_estimator.pt`
  - 카메라 이미지 기반 phase/pose 보조 추정 모델
- `final/vision_metrics.json`
  - 위 vision 모델 평가 지표
- `final/phase_policy.zip`
  - 현재 쓰는 PPO high-level policy 모델
- `final/policy_metadata.json`
  - PPO 학습 설정 메타데이터
- `final/policy_training_summary.json`
  - PPO 학습 요약

## 삭제한 항목

재생성 가능한 항목은 공유 용량 절감 목적으로 삭제.

- MuJoCo vision dataset
- vision evaluation overlay/result
- 이전 vision estimator 모델
- 이전 PPO 모델
- PPO checkpoint 중간 저장본
- real sensor fusion recorder 샘플
- sim diagnostics frame/debug 출력

## 현재 real action bridge 기본 모델 경로

```bash
--vision-model outputs/final/vision_estimator.pt
--policy-model outputs/final/phase_policy.zip
```

## 경로 규칙

- `outputs/final/*`: workspace root 기준 상대경로
- 권장 실행 위치: `~/idle_ws`
- 다른 위치에서 실행: 절대경로 사용
- `package://` 경로: 현재 CLI 모델 인자에서 미사용
