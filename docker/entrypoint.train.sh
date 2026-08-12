#!/bin/bash

set -euo pipefail

readonly repo_root=/workspace/sh_dog
readonly artifacts_root="${repo_root}/artifacts"
readonly training_artifacts="${artifacts_root}/training"
readonly artifacts_gid="$(stat -c '%g' "${artifacts_root}")"

export GIT_CONFIG_COUNT=1
export GIT_CONFIG_KEY_0=safe.directory
export GIT_CONFIG_VALUE_0="${repo_root}"

install -d -m 2775 -g "${artifacts_gid}" \
    "${training_artifacts}" \
    "${training_artifacts}/logs" \
    "${training_artifacts}/outputs"

umask 0002
cd "${training_artifacts}"

if [[ $# -eq 0 ]]; then
    exec /bin/bash
fi

exec /bin/bash -lc "$*"
