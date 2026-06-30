# MuJoCo Phase RL Final CLI

자연어 또는 마이크 명령 → task routing → PPO policy 선택 → 시뮬/실제 로봇 실행 흐름.

비전 추정과 phase fusion 구조는
[`VISION_ESTIMATOR_ARCHITECTURE.ko.md`](VISION_ESTIMATOR_ARCHITECTURE.ko.md)를 기준으로 본다.

## 0. 공통 준비

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

필수 Python 의존성 확인:

```bash
python3 -c "import torch, torchvision, cv2, sounddevice, scipy; from faster_whisper import WhisperModel; print('ok')"
```

누락 시 설치 예:

```bash
python3 -m pip install --user torchvision sounddevice scipy faster-whisper
```

## 1. 빠른 실행 CLI

### 1-A. STT → 시뮬 PPO

마이크로 명령을 받고, task routing 후 MuJoCo 시뮬에서 PPO 실행:

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mujoco_phase_rl sim_intent_runner \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --episodes 3 \
  --steps 16 \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --stack-target-colors red,green,blue \
  --pose-source noisy_gt \
  --pose-noise-std 0.02 \
  --target-noise-std 0.007 \
  --pose-dropout-prob 0.12 \
  --viewer \
  --viewer-slowdown 3.0 \
  --viewer-pause-s 0.5
```

입력 방식:

- `Space`: 녹음 시작
- 말하기
- `Space`: 녹음 종료
- `mic_transcript`, `semantic`, `route`, `policy_model` 확인

### 1-B. STT → 실제 Action Bridge 검증 버전

마이크로 명령을 받고, 실제 카메라 stage1/colorNet 좌표와 RGB basket PPO를 연결. 실제 명령 publish 없음:

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mujoco_phase_rl real_intent_action_bridge \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --vision-source direct \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --direct-vision-rate 5 \
  --present-threshold 0.35 \
  --direct-temporal-tracking \
  --direct-track-max-jump 0.18 \
  --direct-track-hold-frames 5 \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --vision-model outputs/rgb_multicolor_v1/vision_estimator/vision_estimator.pt \
  --device cpu \
  --phase-prior-weight 1.0 \
  --object-memory-timeout 8.0 \
  --target-memory-jump-tolerance 0.04 \
  --phase-hold-timeout 10.0 \
  --home-tolerance 0.25 \
  --target-radius 0.10 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.27 \
  --grasp-z 0.13 \
  --carry-z 0.27 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.33 \
  --home-mode timeout \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5
```

### 1-C. STT → 실제 Action Bridge Armed 버전

Dry-run에서 `semantic`, `route`, `policy_model`, target 좌표, plan 상태 확인 후 사용:

```bash
cd ~/idle_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mujoco_phase_rl real_intent_action_bridge \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --vision-source direct \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --direct-vision-rate 5 \
  --present-threshold 0.35 \
  --direct-temporal-tracking \
  --direct-track-max-jump 0.18 \
  --direct-track-hold-frames 5 \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --vision-model outputs/rgb_multicolor_v1/vision_estimator/vision_estimator.pt \
  --device cpu \
  --phase-prior-weight 1.0 \
  --object-memory-timeout 8.0 \
  --target-memory-jump-tolerance 0.04 \
  --phase-hold-timeout 10.0 \
  --home-tolerance 0.25 \
  --target-radius 0.10 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.27 \
  --grasp-z 0.13 \
  --carry-z 0.27 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.33 \
  --home-mode timeout \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5 \
  --armed
```

실제 로봇 필수 노드:

```bash
ros2 run can_interface can_bridge_node
ros2 run phy plan_node
ros2 run phy plan_compute_node
ros2 run phy gripper_node
```

## 2. 최종 CLI 구조

- `sim_intent_runner`
  - 자연어/STT 명령을 받아 MuJoCo 시뮬에서 PPO policy 실행
  - `--viewer`로 3D 시뮬 직접 확인

- `real_intent_action_bridge`
  - 자연어/STT 명령을 받아 실제 카메라 stage1/colorNet 좌표 추론
  - task에 맞는 PPO policy 자동 선택
  - 기존 `real_action_bridge` 안전 로직 재사용
  - 기본 dry-run, `--armed` 추가 시 실제 `/ee_target`, gripper, home 명령 출력

- 입력 방식
  - `--text "빨간 블록을 바구니에 넣어줘"`
  - `--mic`
  - `--semantic-json '{"success": true, ...}'`

- 마이크 backend
  - 기본값: `--mic-backend auto`
  - 기본 녹음: 스페이스바 시작/스페이스바 종료
  - 고정 시간 녹음: `--mic-duration 5`처럼 초 단위 지정
  - 빈 transcript면 `--mic-empty-retries 2` 기준으로 다시 녹음

- 정책 선택 우선순위
  - `--policy-model` 직접 지정
  - `config/policy_routes.json`의 final policy 경로
  - final 경로가 없으면 `outputs/.../checkpoints`의 최신 checkpoint 자동 선택

## 3. 마이크 명령만 테스트

스페이스바로 시작/종료해서 route만 확인:

```bash
ros2 run mujoco_phase_rl sim_intent_runner \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --episodes 0 \
  --steps 1
```

5초 고정 녹음:

```bash
ros2 run mujoco_phase_rl sim_intent_runner \
  --mic \
  --mic-duration 5 \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --episodes 0 \
  --steps 1
```

## 4. Stage1/colorNet 카메라 단독 테스트

STT/route/PPO 없이 실제 카메라에서 학습된 `stage1_v2 + color_net_v2`만 실행:

```bash
ros2 run mujoco_phase_rl stage1_camera_smoke \
  --camera \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu \
  --present-threshold 0.35 \
  --temporal-tracking \
  --track-max-jump 0.18 \
  --track-hold-frames 5 \
  --loop \
  --rate-hz 5
```

화면으로 같이 보기:

```bash
ros2 run mujoco_phase_rl stage1_camera_smoke \
  --camera \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --device cpu \
  --present-threshold 0.35 \
  --temporal-tracking \
  --track-max-jump 0.18 \
  --track-hold-frames 5 \
  --loop \
  --rate-hz 5 \
  --show
```

## 5. 시뮬 실행

RGB 블록 → 바구니:

```bash
ros2 run mujoco_phase_rl sim_intent_runner \
  --text "빨간 블록을 바구니에 넣어줘" \
  --parser rule \
  --episodes 3 \
  --steps 16 \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --stack-target-colors red,green,blue \
  --pose-source noisy_gt \
  --pose-noise-std 0.02 \
  --target-noise-std 0.007 \
  --pose-dropout-prob 0.12 \
  --viewer \
  --viewer-slowdown 3.0 \
  --viewer-pause-s 0.5
```

RGB 블록 → 다른 블록 위 stack:

```bash
ros2 run mujoco_phase_rl sim_intent_runner \
  --text "빨간 블록을 파란 블록 위에 올려줘" \
  --parser rule \
  --episodes 3 \
  --steps 16 \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --stack-target-colors red,green,blue \
  --pose-source noisy_gt \
  --pose-noise-std 0.02 \
  --target-noise-std 0.007 \
  --pose-dropout-prob 0.12 \
  --viewer \
  --viewer-slowdown 3.0 \
  --viewer-pause-s 0.5
```

마이크로 시뮬 실행:

```bash
ros2 run mujoco_phase_rl sim_intent_runner \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --episodes 3 \
  --steps 16 \
  --object-colors red,green,blue \
  --target-colors red,green,blue \
  --stack-target-colors red,green,blue \
  --viewer
```

## 6. 실제 로봇 실행 준비

기본 노드:

```bash
ros2 run can_interface can_bridge_node
ros2 run phy plan_node
ros2 run phy plan_compute_node
ros2 run phy gripper_node
```

주의:

- `task_fsm_node`는 켜지 않음
- `real_intent_action_bridge`가 task FSM 역할을 대체
- HSV/`idle_vision` 미사용
- stage1/colorNet이 카메라를 직접 열고 Python 내부에서 좌표를 넘김
- 평소 실행에는 `--log-file`을 붙이지 않음
- 디버그 로그 저장이 필요할 때만 `--log-file outputs/real_logs/<name>.log` 추가

카메라 확인:

```bash
ls -l /dev/video*
```

## 7. 실제 로봇 Dry-Run

마이크로 명령 받고, 실제 명령 publish 없이 로그만 확인:

```bash
ros2 run mujoco_phase_rl real_intent_action_bridge \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --vision-source direct \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --direct-vision-rate 5 \
  --present-threshold 0.35 \
  --direct-temporal-tracking \
  --direct-track-max-jump 0.18 \
  --direct-track-hold-frames 5 \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --vision-model outputs/rgb_multicolor_v1/vision_estimator/vision_estimator.pt \
  --device cpu \
  --phase-prior-weight 1.0 \
  --object-memory-timeout 8.0 \
  --target-memory-jump-tolerance 0.04 \
  --phase-hold-timeout 10.0 \
  --home-tolerance 0.25 \
  --target-radius 0.10 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.27 \
  --grasp-z 0.13 \
  --carry-z 0.27 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.33 \
  --home-mode timeout \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5
```

텍스트로 같은 흐름:

```bash
ros2 run mujoco_phase_rl real_intent_action_bridge \
  --text "초록 블록을 바구니에 넣어줘" \
  --parser rule \
  --vision-source direct \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --direct-vision-rate 5 \
  --present-threshold 0.35 \
  --direct-temporal-tracking \
  --direct-track-max-jump 0.18 \
  --direct-track-hold-frames 5 \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --vision-model outputs/rgb_multicolor_v1/vision_estimator/vision_estimator.pt \
  --device cpu \
  --phase-prior-weight 1.0 \
  --object-memory-timeout 8.0 \
  --target-memory-jump-tolerance 0.04 \
  --phase-hold-timeout 10.0 \
  --home-tolerance 0.25 \
  --target-radius 0.10 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.27 \
  --grasp-z 0.13 \
  --carry-z 0.27 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.33 \
  --home-mode timeout \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5
```

## 8. 실제 로봇 Armed

Dry-run에서 아래 항목 확인 후 `--armed` 추가:

- `semantic`의 `action/object/target` 의도 일치
- `route`의 `task_mode` 일치
  - 바구니: `basket`
  - 블록 위 쌓기: `stack`
- `policy_model`이 의도한 checkpoint/final model
- stage1/colorNet 좌표 정상
- `target` 좌표가 로봇 workspace 안
- gripper topic 정상
- plan status 정상

실제 명령 publish:

```bash
ros2 run mujoco_phase_rl real_intent_action_bridge \
  --mic \
  --mic-backend auto \
  --mic-empty-retries 2 \
  --parser rule \
  --vision-source direct \
  --camera-device /dev/video2 \
  --camera-width 1280 \
  --camera-height 720 \
  --direct-vision-rate 5 \
  --present-threshold 0.35 \
  --direct-temporal-tracking \
  --direct-track-max-jump 0.18 \
  --direct-track-hold-frames 5 \
  --policy-model outputs/rgb_multicolor_v1/ppo_policy/final_model.zip \
  --vision-model outputs/rgb_multicolor_v1/vision_estimator/vision_estimator.pt \
  --device cpu \
  --phase-prior-weight 1.0 \
  --object-memory-timeout 8.0 \
  --target-memory-jump-tolerance 0.04 \
  --phase-hold-timeout 10.0 \
  --home-tolerance 0.25 \
  --target-radius 0.10 \
  --target-reached-tolerance 0.12 \
  --yaw-mode fixed \
  --fixed-yaw-deg 0 \
  --pregrasp-z 0.27 \
  --grasp-z 0.13 \
  --carry-z 0.27 \
  --place-z 0.19 \
  --place-xy-mode current \
  --prehome-z 0.33 \
  --home-mode timeout \
  --min-command-period 5.0 \
  --command-timeout 12.0 \
  --log-period 0.5 \
  --armed
```

## 9. 자주 보는 로그

- `mic_transcript=...`
  - Whisper가 인식한 최종 문장

- `semantic`
  - STT/파서가 뽑은 task 의미

- `route`
  - 선택된 task mode와 PPO model

- `REAL ACTION DRY`
  - 실제 publish 없음

- `REAL ACTION ARMED`
  - 실제 publish 활성

- `target ... xyz=...`
  - 다음 `/ee_target` 목표

- `exec inflight`
  - plan 실행 중

- `transaction SUCCESS`
  - 현재 phase command 완료

## 10. 권장 테스트 순서

1. `sim_intent_runner --text`로 basket/stack 각각 확인
2. `sim_intent_runner --mic`로 STT route 확인
3. 실제 로봇 노드 실행
4. `real_intent_action_bridge --text` dry-run
5. `real_intent_action_bridge --mic` dry-run
6. 좌표와 phase 로그 정상 확인
7. `--armed` 추가
