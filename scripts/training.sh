#!/usr/bin/env bash

set -euo pipefail

readonly repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly compose_file="${repo_root}/docker/compose.train.yaml"

usage() {
    cat <<'EOF'
Usage:
  scripts/training.sh train RUN_NAME [options] [-- TRAIN_ARGS...]
  scripts/training.sh eval CHECKPOINT [options] [-- EVAL_ARGS...]
  scripts/training.sh tensorboard [options]

Train options:
  --task TASK              Task name (default: ShDog-Velocity-Flat-v0)
  --num-envs N             Environment count (default: 4096)
  --max-iterations N       Training iterations (default: 10000)
  --logger LOGGER          RSL-RL logger (default: tensorboard)
  --shm-size SIZE          Container shared memory (default: 2gb)

Eval options:
  --task TASK              Evaluation task (default: ShDog-Velocity-Flat-Play-v0)
  --protocol PATH          Replay a repository-relative protocol YAML
  --shm-size SIZE          Container shared memory (default: 2gb)

TensorBoard options:
  --logdir PATH            Repository-relative log directory
                           (default: artifacts/training/logs/rsl_rl/sh_dog_baseline)
  --host HOST              Host bind address (default: 127.0.0.1)
  --port PORT              Host and container port (default: 6006)

Examples:
  scripts/training.sh train baseline_10k
  scripts/training.sh train smoke --num-envs 64 --max-iterations 2
  scripts/training.sh train resumed -- --resume --load_run RUN
  scripts/training.sh eval artifacts/training/logs/rsl_rl/EXPERIMENT/RUN/model.pt
  scripts/training.sh tensorboard --logdir artifacts/training/logs/rsl_rl/other
EOF
}

die() {
    echo "error: $*" >&2
    exit 2
}

require_positive_integer() {
    [[ "$2" =~ ^[1-9][0-9]*$ ]] || die "$1 must be a positive integer: $2"
}

shell_join() {
    local result
    printf -v result '%q ' "$@"
    printf '%s' "${result% }"
}

run_train() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        return
    fi
    [[ $# -gt 0 ]] || die "train requires RUN_NAME"

    local run_name="$1"
    local task="ShDog-Velocity-Flat-v0"
    local num_envs=4096
    local max_iterations=10000
    local logger="tensorboard"
    local shm_size="${SH_DOG_SHM_SIZE:-2gb}"
    local -a extra_args=()
    shift

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --task|--num-envs|--max-iterations|--logger|--shm-size)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                case "$1" in
                    --task) task="$2" ;;
                    --num-envs) num_envs="$2" ;;
                    --max-iterations) max_iterations="$2" ;;
                    --logger) logger="$2" ;;
                    --shm-size) shm_size="$2" ;;
                esac
                shift 2
                ;;
            --)
                shift
                extra_args=("$@")
                break
                ;;
            -h|--help)
                usage
                return
                ;;
            *) die "unknown train option: $1" ;;
        esac
    done

    [[ -n "${run_name}" && "${run_name}" != -* ]] || die "invalid RUN_NAME: ${run_name}"
    [[ -n "${task}" ]] || die "task must not be empty"
    [[ -n "${logger}" ]] || die "logger must not be empty"
    [[ -n "${shm_size}" ]] || die "shm-size must not be empty"
    require_positive_integer "num-envs" "${num_envs}"
    require_positive_integer "max-iterations" "${max_iterations}"

    local -a command=(
        /workspace/isaaclab/isaaclab.sh -p
        /workspace/sh_dog/training/scripts/rsl_rl/train.py
        --task "${task}"
        --headless
        --num_envs "${num_envs}"
        --max_iterations "${max_iterations}"
        --run_name "${run_name}"
        --logger "${logger}"
        "${extra_args[@]}"
    )

    echo "task=${task} run_name=${run_name} num_envs=${num_envs} max_iterations=${max_iterations} logger=${logger}"
    SH_DOG_SHM_SIZE="${shm_size}" docker compose -f "${compose_file}" run \
        --rm train "$(shell_join "${command[@]}")"
}

resolve_repo_file() {
    local path="$1"
    if [[ "${path}" == /workspace/sh_dog/* ]]; then
        path="${repo_root}/${path#/workspace/sh_dog/}"
    elif [[ "${path}" != /* ]]; then
        path="${repo_root}/${path}"
    fi

    path="$(realpath -m -- "${path}")"
    [[ "${path}" == "${repo_root}"/* ]] || die "path must stay inside the repository: $1"
    [[ -f "${path}" ]] || die "file does not exist: ${path}"
    printf '/workspace/sh_dog/%s' "${path#"${repo_root}/"}"
}

run_eval() {
    if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
        usage
        return
    fi
    [[ $# -gt 0 ]] || die "eval requires CHECKPOINT"

    local checkpoint
    checkpoint="$(resolve_repo_file "$1")"
    local task="ShDog-Velocity-Flat-Play-v0"
    local protocol=""
    local shm_size="${SH_DOG_SHM_SIZE:-2gb}"
    local -a extra_args=()
    shift

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --task|--protocol|--shm-size)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                case "$1" in
                    --task) task="$2" ;;
                    --protocol) protocol="$(resolve_repo_file "$2")" ;;
                    --shm-size) shm_size="$2" ;;
                esac
                shift 2
                ;;
            --)
                shift
                extra_args=("$@")
                break
                ;;
            -h|--help)
                usage
                return
                ;;
            *) die "unknown eval option: $1" ;;
        esac
    done

    [[ -n "${task}" ]] || die "task must not be empty"
    [[ -n "${shm_size}" ]] || die "shm-size must not be empty"

    local -a command=(
        /workspace/isaaclab/isaaclab.sh -p
        /workspace/sh_dog/training/scripts/rsl_rl/eval.py
        --task "${task}"
        --checkpoint "${checkpoint}"
        --headless
    )
    if [[ -n "${protocol}" ]]; then
        command+=(--protocol "${protocol}")
    fi
    command+=("${extra_args[@]}")

    echo "task=${task} checkpoint=${checkpoint} protocol=${protocol:-generated}"
    SH_DOG_SHM_SIZE="${shm_size}" docker compose -f "${compose_file}" run \
        --rm train "$(shell_join "${command[@]}")"
}

run_tensorboard() {
    local logdir="artifacts/training/logs/rsl_rl/sh_dog_baseline"
    local host="127.0.0.1"
    local port=6006

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --logdir|--host|--port)
                [[ $# -ge 2 ]] || die "$1 requires a value"
                case "$1" in
                    --logdir) logdir="$2" ;;
                    --host) host="$2" ;;
                    --port) port="$2" ;;
                esac
                shift 2
                ;;
            -h|--help)
                usage
                return
                ;;
            *) die "unknown tensorboard option: $1" ;;
        esac
    done

    [[ -n "${host}" ]] || die "host must not be empty"
    require_positive_integer "port" "${port}"
    (( port <= 65535 )) || die "port must not exceed 65535: ${port}"

    if [[ "${logdir}" == /workspace/sh_dog/* ]]; then
        logdir="${logdir#/workspace/sh_dog/}"
    elif [[ "${logdir}" == /* ]]; then
        die "logdir must be repository-relative or below /workspace/sh_dog"
    fi

    local host_logdir
    host_logdir="$(realpath -m -- "${repo_root}/${logdir}")"
    [[ "${host_logdir}" == "${repo_root}"/* ]] || die "logdir must stay inside the repository: ${logdir}"
    [[ -d "${host_logdir}" ]] || die "logdir does not exist: ${host_logdir}"

    local container_logdir="/workspace/sh_dog/${host_logdir#"${repo_root}/"}"
    local -a command=(
        /workspace/isaaclab/isaaclab.sh -p -m tensorboard.main
        --logdir "${container_logdir}"
        --host 0.0.0.0
        --port "${port}"
    )

    echo "logdir=${host_logdir} url=http://${host}:${port}"
    docker compose -f "${compose_file}" run --rm \
        -p "${host}:${port}:${port}" train "$(shell_join "${command[@]}")"
}

case "${1:-}" in
    train)
        shift
        run_train "$@"
        ;;
    eval)
        shift
        run_eval "$@"
        ;;
    tensorboard)
        shift
        run_tensorboard "$@"
        ;;
    -h|--help)
        usage
        ;;
    "")
        usage
        exit 2
        ;;
    *) die "unknown command: $1" ;;
esac
