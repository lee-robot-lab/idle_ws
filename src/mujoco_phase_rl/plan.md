방향은 이렇게 잡는 게 맞아.

  지금 상황 정리:

  grasp z 하강 반영됨
  scripted rollout 성공
  새 vision dataset 생성됨
  vision estimator 학습/평가함
  vision phase 성능은 낮아짐
  하지만 object/target/ee 위치 추정은 쓸 만함
  PPO graspdown 버전은 아직 새로 안 끝낸 상태

  다음 순서는 이게 맞음.

  1. graspdown 반영된 env로 PPO를 먼저 학습한다
  2. PPO clean/noisy 평가로 task-level RL이 아직 되는지 확인한다
  3. 그 다음 fused observation/phase estimator를 구현한다
  4. fused observation을 MuJoCo에서 먼저 diagnostic한다
  5. 필요하면 fused observation 기반으로 PPO를 다시 학습한다
  6. 마지막에 실제 로봇 topic/camera를 read-only로 연결한다

  즉 지금 당장 할 건:

  PPO graspdown 학습 먼저

  왜냐하면 grasp z를 바꾸면 action 결과/reward/trajectory가 바뀌었기 때문에, 현재 env에서 RL이 다시 잘 되는지 먼저 확인해야 함.

  그 다음에 fused 쪽으로 가야 해. 순서를 바꾸면 헷갈림.

  추천 흐름:

  A. 현재 env가 task를 풀 수 있나?
     -> PPO graspdown 학습/평가

  B. vision 단독 phase가 약한 문제를 어떻게 보완할까?
     -> fused phase estimator 구현

  C. fused observation으로 RL을 다시 학습할 필요가 있나?
     -> sim에서 fused/noisy observation으로 evaluate/train

  D. 실제 입력에 붙일 수 있나?
     -> real_phase_diagnostics read-only

  구체적으로는:

  Step 1: PPO 학습
  outputs/train/ppo_graspdown_50k

  Step 2: 평가
  clean / noisy pose

  Step 3: fused phase estimator 추가
  vision_pred + robot_state + gripper_state + history -> fused_phase

  Step 4: sim_phase_diagnostics에 fused_phase 표시
  gt_phase / vision_phase / fused_phase / policy_action 비교

  Step 5: PPO observation에 fused phase 또는 fused state 넣는 옵션 추가
  pose-source=fused_sim 같은 식

  Step 6: 필요하면 PPO 재학습
  outputs/ppo_fused_graspdown_50k

  Step 7: real_phase_diagnostics 구현
  실제 camera + motor_state + gripper_state + vision estimator + policy
  단, command publish 안 함

  결론:

  지금은 PPO graspdown 학습부터 하는 게 맞다.
  그 다음 fused phase/observation 구현.
  그 다음 fused 기반 RL 재학습 여부 판단.
  그 다음 실제 센서 read-only bridge.

  바로 실행할 PPO 명령은:

  ros2 run mujoco_phase_rl train_ppo \
    --output-dir outputs/train/ppo_graspdown_50k \
    --total-timesteps 50000 \
    --n-envs 4 \
    --max-episode-steps 16 \
    --n-steps 64 \
    --batch-size 128 \
    --gamma 0.95 \
    --learning-rate 3e-4 \
    --device cpu

  끝나면:

  ros2 run mujoco_phase_rl evaluate_policy \
    --model outputs/train/ppo_graspdown_50k/final_model.zip \
    --episodes 100 \
    --steps 16 \
    --json

  ros2 run mujoco_phase_rl evaluate_policy \
    --model outputs/train/ppo_graspdown_50k/final_model.zip \
    --episodes 100 \
    --steps 16 \
    --pose-source noisy_gt \
    --pose-noise-std 0.015 \
    --target-noise-std 0.005 \
    --pose-dropout-prob 0.10 \
    --json

  이 결과 보고 다음 구현 들어가면 됨.
