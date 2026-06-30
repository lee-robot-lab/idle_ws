공부할 코드 순서:

  1. Phase / Command 구조
     src/mujoco_phase_rl/mujoco_phase_rl/tasks/phase_manager.py:6

     여기서 OBSERVE_OBJECT, GRASP, LIFT, MOVE_TO_PLACE, PLACE, RETREAT 같은 phase랑 각 phase에서 허용되는 command가 정의됨. PPO가 아무거나 하는 게 아니라 ALLOWED_COMMANDS 안
     에서 움직임.

  2. 시뮬 환경 핵심
     src/mujoco_phase_rl/mujoco_phase_rl/envs/phase_pick_place_env.py:62

     제일 중요함. 여기서 PPO action이 실제 시뮬 동작으로 바뀜.

     특히 볼 함수:
      - _execute_move_to_pregrasp 물체 위로 이동
      - _execute_grasp 직선 하강 후 그리퍼 닫기
      - _execute_lift 잡은 물체 상승
  3. 보상 함수
     src/mujoco_phase_rl/mujoco_phase_rl/tasks/reward.py:21


     stack_xy_accuracy, stack_height_accuracy, stack_on_target 같은 게 지금 stacking 학습의 핵심 보상임.

  4. 학습 실행 코드
     src/mujoco_phase_rl/mujoco_phase_rl/policies/train_ppo.py:12

     CLI 인자, PPO(...), resume, checkpoint 저장이 여기 있음.
     방금 고친 learning rate resume도 여기.

  5. 평가 코드
     evaluate_policy, sim_phase_diagnostics

     결과 JSON에서 success_rate만 보지 말고:
      - GRASP_FAIL
      - WORKSPACE_FAIL
      - phase_failure_count
      - planner_fail_reason_counts

     이것까지 같이 봐야 함.

  정리하면, 지금 네가 제일 먼저 봐야 할 건 phase_pick_place_env.py랑 reward.py임.
  이 두 개 이해하면 “왜 얘가 이렇게 움직이고, 왜 그 행동이 보상 받는지”가 보임.
