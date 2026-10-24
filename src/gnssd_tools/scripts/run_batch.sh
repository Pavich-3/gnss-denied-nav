#!/usr/bin/env bash
#
# Run the same scenario for a range of seeds, one recorded run per seed.
#
# Usage (from the repository root):
#   src/gnssd_tools/scripts/run_batch.sh <world> <scenario> <laps> <seed_from> <seed_to> [extra launch args]
# Examples:
#   src/gnssd_tools/scripts/run_batch.sh room room_loop 2 1 3
#   src/gnssd_tools/scripts/run_batch.sh room room_long 10 1 1
#   src/gnssd_tools/scripts/run_batch.sh warehouse warehouse_loop 1 1 3
#   src/gnssd_tools/scripts/run_batch.sh room room_loop 2 1 1 odom_noise:=false
#
# For every seed it
#   1. refuses to start if any ROS node is still running (a leftover node
#      would publish into the same topics and corrupt the bag);
#   2. runs `ros2 launch gnssd_bringup run.launch.py` (which itself refuses
#      to run with uncommitted git changes unless allow_dirty:=true is given);
#   3. appends one line to bags/index.csv:
#      bag, world, scenario, laps, seed, commit, duration_s, extra_args
#
# The launch output of every run is kept in bags/logs/.

if [ $# -lt 5 ]; then
  echo "Usage: $0 <world> <scenario> <laps> <seed_from> <seed_to> [extra launch args]"
  exit 1
fi

WORLD=$1
SCENARIO=$2
LAPS=$3
SEED_FROM=$4
SEED_TO=$5
shift 5
EXTRA_ARGS="$*"

REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
cd "$REPO" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
# Treat unset variables as errors from here on (ROS setup.bash itself does
# not work with `set -u`, so it is switched on only after sourcing).
set -u

INDEX=bags/index.csv
LOG_DIR=bags/logs
mkdir -p "$LOG_DIR"
if [ ! -f "$INDEX" ]; then
  echo "bag,world,scenario,laps,seed,commit,duration_s,extra_args" > "$INDEX"
fi

# One lap takes ~107 s; allow generous extra time for start-up and shutdown.
TIMEOUT=$((LAPS * 150 + 180))

for SEED in $(seq "$SEED_FROM" "$SEED_TO"); do
  echo "=== $WORLD / $SCENARIO, laps $LAPS, seed $SEED, $(date +%T)"

  NODES=$(ros2 node list --no-daemon --spin-time 3 2>/dev/null)
  if [ -n "$NODES" ]; then
    echo "ABORT: ROS nodes are still running:"
    echo "$NODES"
    exit 1
  fi

  LOG="$LOG_DIR/${SCENARIO}_seed${SEED}_$(date +%Y%m%d_%H%M%S).log"
  # shellcheck disable=SC2086
  timeout "$TIMEOUT" ros2 launch gnssd_bringup run.launch.py \
    world:="$WORLD" scenario:="$SCENARIO" seed:="$SEED" laps:="$LAPS" $EXTRA_ARGS > "$LOG" 2>&1
  STATUS=$?

  BAG=$(grep -oE 'Recording to [^ ]+' "$LOG" | head -1 | cut -d' ' -f3)
  if [ $STATUS -ne 0 ] || [ -z "$BAG" ] || ! grep -q 'Route finished' "$LOG"; then
    echo "ABORT: run failed (exit $STATUS), see $LOG"
    grep -E 'ERROR|refusing' "$LOG" | head -5
    exit 1
  fi
  grep -E 'Route finished' "$LOG" | sed 's/.*\] //'

  COMMIT=$(grep -E '^git_commit:' "$BAG/run_info.yaml" | awk '{print $2}')
  DURATION_NS=$(grep -m1 -A1 -E '^  duration:' "$BAG/metadata.yaml" | grep -oE '[0-9]+' | tail -1)
  DURATION=$(awk "BEGIN {printf \"%.1f\", $DURATION_NS / 1e9}")
  echo "$(basename "$BAG"),$WORLD,$SCENARIO,$LAPS,$SEED,$COMMIT,$DURATION,$EXTRA_ARGS" >> "$INDEX"

  sleep 5  # let the processes of this run disappear before the node check
done

echo "=== batch done, index: $INDEX"
