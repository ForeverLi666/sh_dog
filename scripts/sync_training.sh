#!/usr/bin/env bash

set -euo pipefail

readonly repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly marker=".sh_dog_sync_root"
readonly config_file="${repo_root}/scripts/sync_training.conf"

if [[ $# -ne 0 ]]; then
    echo "usage: $0" >&2
    exit 2
fi

# shellcheck source=sync_training.conf
source "${config_file}"

readonly remote_host="${SH_DOG_TRAIN_HOST}"
readonly remote_dir="${SH_DOG_TRAIN_DIR}"

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
        echo "generated USD is missing; run scripts/training.sh build-usd: ${relative_path}" >&2
        exit 1
    fi
done

ssh "${remote_host}" bash -s -- "${remote_dir}" "${marker}" <<'REMOTE'
set -euo pipefail
remote_dir="$1"
marker="$2"

if [[ ! -f "${remote_dir}/${marker}" ]]; then
    if [[ -e "${remote_dir}" && ! -d "${remote_dir}" ]]; then
        echo "sync destination is not a directory: ${remote_dir}" >&2
        exit 1
    fi
    if [[ -d "${remote_dir}" ]] && find "${remote_dir}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
        echo "refusing to initialize a non-empty directory without ${marker}: ${remote_dir}" >&2
        exit 1
    fi
    mkdir -p "${remote_dir}/artifacts"
    touch "${remote_dir}/${marker}"
else
    mkdir -p "${remote_dir}/artifacts"
fi
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

rsync "${rsync_args[@]}" "${repo_root}/" "${remote_host}:${remote_dir%/}/"
