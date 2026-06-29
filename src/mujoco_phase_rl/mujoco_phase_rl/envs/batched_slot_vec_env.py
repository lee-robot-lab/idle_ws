# ================================================================
# batched_slot_vec_env.py
# 설명: 4개 env의 SlotEmbedder 추론을 배치로 묶어 GPU 효율을 높이는 VecEnv.
#       DummyVecEnv를 상속하며 step_wait()만 오버라이드한다.
# 사용법: from mujoco_phase_rl.envs.batched_slot_vec_env import BatchedSlotDummyVecEnv
# ================================================================
from __future__ import annotations

from copy import deepcopy

import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv


class BatchedSlotDummyVecEnv(DummyVecEnv):
    """DummyVecEnv를 상속해 slot 추론을 배치로 처리.

    step_wait() 흐름:
      1. 각 env에 deferred 모드 설정 → step() 시 NN 생략, 이미지만 렌더
      2. done env는 deferred 해제 후 reset (정상 추론)
      3. 살아있는 env의 이미지를 배치로 모아 NN 1회 forward
      4. 결과 주입 후 _observe() 재호출 → 신선한 obs로 buf_obs 갱신
    """

    def step_wait(self):
        # 1. 모든 env deferred 모드 활성화
        for env in self.envs:
            env._slot_embed_deferred = True

        alive_indices = []  # done 아닌 env 인덱스

        for env_idx in range(self.num_envs):
            env = self.envs[env_idx]
            obs, self.buf_rews[env_idx], terminated, truncated, self.buf_infos[env_idx] = (
                env.step(self.actions[env_idx])
            )
            done = terminated or truncated
            self.buf_dones[env_idx] = done
            self.buf_infos[env_idx]["TimeLimit.truncated"] = truncated and not terminated

            if done:
                # terminal obs는 stale embedding이지만 허용 범위 (에피소드 종료 직전)
                self.buf_infos[env_idx]["terminal_observation"] = obs
                env._slot_embed_deferred = False
                obs, self.reset_infos[env_idx] = env.reset()
            else:
                alive_indices.append(env_idx)

            self._save_obs(env_idx, obs)

        # 2. 살아있는 env의 pending img 수집
        if alive_indices:
            imgs, embedders = [], []
            valid_indices = []
            for env_idx in alive_indices:
                env = self.envs[env_idx]
                if env._slot_embed_pending_img is not None:
                    imgs.append(env._slot_embed_pending_img)
                    embedders.append(env.slot_embedder)
                    valid_indices.append(env_idx)

            if imgs:
                # 3. 배치 NN 추론 (ref는 첫 번째 embedder의 가중치 사용)
                from mujoco_phase_rl.perception.image_embedding import SlotEmbedder

                batch_results = SlotEmbedder.batch_embed_from_prerendered(
                    embedders[0], embedders, imgs
                )

                # 4. 결과 주입 후 obs 재계산
                for list_idx, env_idx in enumerate(valid_indices):
                    env = self.envs[env_idx]
                    emb, curr_slots = batch_results[list_idx]
                    env.inject_slot_result(emb, curr_slots)
                    fresh_obs = env._observe()
                    self._save_obs(env_idx, fresh_obs)

        return (
            self._obs_from_buf(),
            np.copy(self.buf_rews),
            np.copy(self.buf_dones),
            deepcopy(self.buf_infos),
        )
