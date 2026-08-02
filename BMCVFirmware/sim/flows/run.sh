#!/usr/bin/env bash
# Replay every input flow through the simulator and compare against its golden
# output. These cover whole interactions - hold a shift button, tap a scene,
# turn an encoder - which unit tests cannot express.
#
#   run.sh check   diff against the goldens (default; non-zero on mismatch)
#   run.sh bless   overwrite the goldens with current behaviour
set -uo pipefail

cd "$(dirname "$0")/../.."
CLI=./build-sim/bmcv_sim_cli
MODE="${1:-check}"

if [ ! -x "$CLI" ]; then
  echo "missing $CLI - run 'just configure-sim && just sim'" >&2
  exit 1
fi

fail=0
for script in sim/flows/*.txt; do
  name=$(basename "$script" .txt)
  golden="sim/flows/golden/$name.csv"

  # Each flow may declare its own CLI arguments on a line starting with "#!args".
  args=$(sed -n 's/^#!args //p' "$script")

  # shellcheck disable=SC2086
  out=$("$CLI" --script="$script" $args) || { echo "FAIL $name (cli error)"; fail=1; continue; }

  if [ "$MODE" = "bless" ]; then
    mkdir -p sim/flows/golden
    printf '%s\n' "$out" > "$golden"
    echo "blessed $name"
    continue
  fi

  if [ ! -f "$golden" ]; then
    echo "FAIL $name (no golden; run 'just flows-bless')"
    fail=1
    continue
  fi

  if printf '%s\n' "$out" | diff -q - "$golden" >/dev/null; then
    echo "ok   $name"
  else
    echo "FAIL $name"
    printf '%s\n' "$out" | diff -u "$golden" - | head -20
    fail=1
  fi
done

exit $fail
