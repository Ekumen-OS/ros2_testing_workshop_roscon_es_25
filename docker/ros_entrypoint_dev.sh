#!/usr/bin/env bash

# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

# runtime-friendly defaults
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
USERNAME="${USERNAME:-developer}"
WORKSPACE="/home/${USERNAME}/ws"
SRCDIR="${WORKSPACE}/src"

# run rosdep if src contains packages
if [ -d "${SRCDIR}" ] && [ "$(ls -A "${SRCDIR}" 2>/dev/null || true)" ]; then
  echo "[entrypoint] detected packages in ${SRCDIR}"
  echo "[entrypoint] running: sudo rosdep install --from-paths \"${SRCDIR}\" --ignore-src -r -y --rosdistro \"${ROS_DISTRO}\""
  if ! sudo rosdep install --from-paths "${SRCDIR}" --ignore-src -r -y --rosdistro "${ROS_DISTRO}"; then
    echo "[entrypoint] warning: rosdep install failed. You can retry manually inside the container."
  fi
else
  echo "[entrypoint] no packages found in ${SRCDIR}; skipping rosdep install"
fi

# If no args were provided to the container, default to an interactive bash shell.
# This is robust whether CMD is absolute or relative.
if [ "$#" -eq 0 ]; then
  # prefer /bin/bash absolute path
  set -- /bin/bash
fi

exec "$@"
