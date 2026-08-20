#!/usr/bin/env bash
set -euo pipefail

# Installs apt packages on a GitHub-hosted runner, but only the ones the image
# does not already ship, and never for longer than it takes to notice the
# mirror is wedged. Takes `command:package` pairs, so each job names the tool
# it actually needs rather than a package list nobody rechecks:
#
#   scripts/ci-apt.sh ninja:ninja-build arm-none-eabi-gcc:gcc-arm-none-eabi
#
# This exists because of the 2026-08-18 stall. `apt-get update` stopped
# mid-fetch on archive.ubuntu.com and simply never returned, so every job that
# opened with `apt-get update && apt-get install` sat there until Actions
# killed it at the 6h ceiling - CI, Pages and the Rack plugin all at once, none
# of them failing fast enough to say what was wrong. Two things went wrong and
# both are fixed here: the jobs installed packages the image already had, and
# nothing bounded the wait.
#
# apt's own Acquire::*::Timeout only fires on a connection that has gone away.
# The stall was a live connection that stopped delivering bytes, which those
# never catch - `timeout` is what actually bounds it. ForceIPv4 is the other
# half: the Azure mirror's IPv6 route is the usual source of these hangs, and
# the runner has no working IPv6 path to it anyway.

missing=()
for pair in "$@"; do
  cmd=${pair%%:*}
  pkg=${pair#*:}
  if command -v "$cmd" >/dev/null 2>&1; then
    echo "$cmd: already on the image"
  else
    echo "$cmd: missing, will install $pkg"
    missing+=("$pkg")
  fi
done

if [ ${#missing[@]} -eq 0 ]; then
  echo "nothing to install"
  exit 0
fi

opts=(-o Acquire::Retries=3
      -o Acquire::http::Timeout=20
      -o Acquire::https::Timeout=20
      -o Acquire::ForceIPv4=true)

for attempt in 1 2 3; do
  if timeout 180 sudo apt-get update -qq "${opts[@]}" &&
     timeout 300 sudo apt-get install -y -qq --no-install-recommends \
       "${opts[@]}" "${missing[@]}"; then
    exit 0
  fi
  echo "apt attempt $attempt/3 failed or timed out" >&2
  if [ "$attempt" -lt 3 ]; then
    sleep $((attempt * 10))
  fi
done

echo "apt could not install: ${missing[*]}" >&2
exit 1
