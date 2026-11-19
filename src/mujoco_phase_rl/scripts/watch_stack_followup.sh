#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

BASE_OUTPUT_DIR="${BASE_OUTPUT_DIR:-outputs/ppo_stack_base_s0}"
BASE_MODEL="${BASE_MODEL:-${BASE_OUTPUT_DIR}/final_model.zip}"
BG_IMAGE="${BG_IMAGE:-../../data/background.jpg}"
POLL_SECONDS="${POLL_SECONDS:-120}"

EVAL_MAX_SCENES="${EVAL_MAX_SCENES:-5}"
EVAL_STEPS="${EVAL_STEPS:-64}"
TASKS="${TASKS:-pick_place,stack}"
BLOCK_COLORS="${BLOCK_COLORS:-red,green,blue}"

PG_OUTPUT_DIR="${PG_OUTPUT_DIR:-outputs/ppo_stack_pg_s0}"
PG_TIMESTEPS="${PG_TIMESTEPS:-100000}"
PG_LEARNING_RATE="${PG_LEARNING_RATE:-1e-4}"

RUN_ROBUST="${RUN_ROBUST:-0}"
ROBUST_OUTPUT_DIR="${ROBUST_OUTPUT_DIR:-outputs/ppo_stack_robust_s0}"
ROBUST_TIMESTEPS="${ROBUST_TIMESTEPS:-100000}"
ROBUST_LEARNING_RATE="${ROBUST_LEARNING_RATE:-1e-4}"
ROBUST_AUG_PROB="${ROBUST_AUG_PROB:-0.3}"
ROBUST_PERTURB_PROB="${ROBUST_PERTURB_PROB:-0.01}"
ROBUST_PERTURB_MAX="${ROBUST_PERTURB_MAX:-0.05}"

WATCH_LOG="${WATCH_LOG:-outputs/ppo_stack_followup_watch.log}"
mkdir -p outputs "${BASE_OUTPUT_DIR}" "${PG_OUTPUT_DIR}"
exec > >(tee -a "${WATCH_LOG}") 2>&1

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*"
}

latest_checkpoint() {
  find "${BASE_OUTPUT_DIR}/checkpoints" \
    -maxdepth 1 \
    -type f \
    -name 'ppo_stack_*_steps.zip' \
    -printf '%T@ %f\n' 2>/dev/null \
    | sort -n \
    | tail -1 \
    || true
}

run_eval() {
  local pose_source="$1"
  local out_json="$2"

  log "eval start pose_source=${pose_source} out=${out_json}"
  python3 mujoco_phase_rl/policies/run_val_sim_batch.py \
    --model "${BASE_MODEL}" \
    --bg-image "${BG_IMAGE}" \
    --tasks "${TASKS}" \
    --block-colors "${BLOCK_COLORS}" \
    --max-scenes "${EVAL_MAX_SCENES}" \
    --steps "${EVAL_STEPS}" \
    --pose-source "${pose_source}" \
    --no-augment \
    --out "${out_json}"
  log "eval done pose_source=${pose_source} out=${out_json}"
}

log "watcher started project_root=${PROJECT_ROOT}"
log "waiting for base model: ${BASE_MODEL}"

while [ ! -f "${BASE_MODEL}" ]; do
  latest="$(latest_checkpoint)"
  log "base still running; latest_checkpoint=${latest:-none}"
  sleep "${POLL_SECONDS}"
done

log "base model found: ${BASE_MODEL}"

run_eval gt "${BASE_OUTPUT_DIR}/eval_base_gt_val${EVAL_MAX_SCENES}.json"
run_eval slot "${BASE_OUTPUT_DIR}/eval_base_slot_val${EVAL_MAX_SCENES}.json"

log "perception-grounded finetune start output=${PG_OUTPUT_DIR}"
python3 mujoco_phase_rl/policies/finetune_stack_robust.py \
  --base-model "${BASE_MODEL}" \
  --output-dir "${PG_OUTPUT_DIR}" \
  --pose-source slot \
  --no-aug-slot \
  --perturb-prob 0.0 \
  --aug-prob 0.0 \
  --total-timesteps "${PG_TIMESTEPS}" \
  --learning-rate "${PG_LEARNING_RATE}"
log "perception-grounded finetune done output=${PG_OUTPUT_DIR}"

if [ "${RUN_ROBUST}" = "1" ]; then
  log "robust finetune start output=${ROBUST_OUTPUT_DIR}"
  mkdir -p "${ROBUST_OUTPUT_DIR}"
  python3 mujoco_phase_rl/policies/finetune_stack_robust.py \
    --base-model "${PG_OUTPUT_DIR}/final_model.zip" \
    --output-dir "${ROBUST_OUTPUT_DIR}" \
    --pose-source slot \
    --aug-prob "${ROBUST_AUG_PROB}" \
    --perturb-prob "${ROBUST_PERTURB_PROB}" \
    --perturb-max "${ROBUST_PERTURB_MAX}" \
    --total-timesteps "${ROBUST_TIMESTEPS}" \
    --learning-rate "${ROBUST_LEARNING_RATE}"
  log "robust finetune done output=${ROBUST_OUTPUT_DIR}"
else
  log "robust finetune skipped; set RUN_ROBUST=1 to enable it"
fi

log "all requested follow-up jobs complete"
