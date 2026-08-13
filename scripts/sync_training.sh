#!/usr/bin/env bash

set -euo pipefail

readonly repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly marker=".sh_dog_sync_root"

initialize=false
dry_run=false

usage() {
    echo "usage: $0 [--init] [--dry-run]"
}

for argument in "$@"; do
    case "${argument}" in
        --init) initialize=true ;;
        --dry-run) dry_run=true ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
done

readonly remote_host="${SH_DOG_TRAIN_HOST:?set SH_DOG_TRAIN_HOST to an SSH host or alias}"
readonly remote_dir="${SH_DOG_TRAIN_DIR:?set SH_DOG_TRAIN_DIR to a dedicated absolute directory}"

if [[ ! "${remote_host}" =~ ^[A-Za-z0-9._@-]+$ ]]; then
    echo "invalid SH_DOG_TRAIN_HOST: ${remote_host}" >&2
    exit 2
fi

if [[ ! "${remote_dir}" =~ ^/[A-Za-z0-9._/-]+/(sh_dog|sh_dog_sync)/?$ ]]; then
    echo "SH_DOG_TRAIN_DIR must be an absolute path ending with /sh_dog or /sh_dog_sync: ${remote_dir}" >&2
    exit 2
fi

required_usd=(
    assets/sh_dog/usd/sh_dog.usd
    assets/sh_dog/usd/configuration/sh_dog_base.usd
    assets/sh_dog/usd/configuration/sh_dog_physics.usd
    assets/sh_dog/usd/configuration/sh_dog_robot.usd
    assets/sh_dog/usd/configuration/sh_dog_sensor.usd
)

for relative_path in "${required_usd[@]}"; do
    if [[ ! -f "${repo_root}/${relative_path}" ]]; then
        echo "generated USD is missing; run python scripts/build_usd.py: ${relative_path}" >&2
        exit 1
    fi
done

if ${initialize}; then
    ssh "${remote_host}" bash -s -- "${remote_dir}" "${marker}" <<'REMOTE'
set -euo pipefail
remote_dir="$1"
marker="$2"

if [[ -d "${remote_dir}" ]] && find "${remote_dir}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
    echo "refusing to initialize a non-empty directory: ${remote_dir}" >&2
    exit 1
fi

mkdir -p "${remote_dir}/artifacts"
touch "${remote_dir}/${marker}"
REMOTE
fi

ssh "${remote_host}" bash -s -- "${remote_dir}" "${marker}" <<'REMOTE'
set -euo pipefail
remote_dir="$1"
marker="$2"

if [[ ! -f "${remote_dir}/${marker}" ]]; then
    echo "sync marker is missing; initialize the dedicated directory with --init" >&2
    exit 1
fi

mkdir -p "${remote_dir}/artifacts"
REMOTE

rsync_args=(
    --archive
    --compress
    --delete-delay
    --human-readable
    --itemize-changes
    "--exclude=/.git/"
    "--exclude=/.sh_dog_sync_root"
    "--exclude=/.vscode/"
    "--exclude=/artifacts/"
    "--exclude=/docker/artifacts/"
    "--exclude=/docker/cluster/exports/"
    "--exclude=/assets/sh_dog/usd/.asset_hash"
    "--exclude=/assets/sh_dog/usd/config.yaml"
    "--exclude=**/__pycache__/"
    "--exclude=**/.pytest_cache/"
    "--exclude=**/*.egg-info/"
    "--exclude=**/build/"
    "--exclude=**/cmake-build*/"
    "--exclude=**/*.pyc"
    "--exclude=**/*.so"
    "--exclude=**/*.log*"
    "--exclude=**/output/"
    "--exclude=**/outputs/"
    "--exclude=**/runs/"
    "--exclude=**/logs/"
    "--exclude=**/recordings/"
    "--exclude=**/videos/"
    "--exclude=**/wandb/"
    "--exclude=**/.neptune/"
    "--exclude=/.pretrained_checkpoints/"
    "--exclude=/datasets/"
    "--exclude=/_isaac_sim*"
    "--exclude=/_repo/"
    "--exclude=/_build/"
)

if ${dry_run}; then
    rsync_args+=(--dry-run)
fi

rsync "${rsync_args[@]}" "${repo_root}/" "${remote_host}:${remote_dir%/}/"
