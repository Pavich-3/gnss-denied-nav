#!/usr/bin/env bash
#
# Step 7: the final experiment series of the thesis (all scenarios, all seeds).
#
# Usage (from the repository root, ideally inside tmux so it survives a closed
# terminal; it keeps the laptop awake by itself with systemd-inhibit):
#   tmux new -s series
#   src/gnssd_tools/scripts/run_final_series.sh
#   (detach: Ctrl-b d, re-attach later: tmux attach -t series)
#
# Resumable: run_batch.sh skips every run that is already complete at the
# current commit, so after an interruption just start this script again.
# It stops at the first failed run (the cause should be looked at first).
#
# Order: the runs that save the maps (room_loop / warehouse_loop seed 1)
# come before the localization runs that use them.
#
# Estimated wall time ~12.5 h, disk ~24.5 GB (bags; Step 7 in docs/progress.md).

REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
cd "$REPO" || exit 1

if [ -z "${GNSSD_SERIES_INHIBITED:-}" ]; then
  # Re-run this script under systemd-inhibit: no sleep, no idle suspend, and
  # closing the lid does not suspend either, until the series ends.
  export GNSSD_SERIES_INHIBITED=1
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch \
    --why="gnss-denied-nav final series" "$0" "$@"
fi

BATCH=src/gnssd_tools/scripts/run_batch.sh
ROOM_MAP=final_room_s1
WAREHOUSE_MAP=final_wh_s1
ROOM_DROPOUT='scan_dropout:=[[140,200]]'        # 60 s in the 2nd lap
WAREHOUSE_DROPOUT='scan_dropout:=[[285,405]]'   # 120 s

run() {
  echo "##### $(date '+%F %T')  $*"
  "$BATCH" "$@" || { echo "##### series stopped: run failed ($*)"; exit 1; }
}

need_map() {
  if [ ! -f "maps/$1.posegraph" ] || [ ! -f "maps/$1.data" ]; then
    echo "##### series stopped: map $1 is missing (its mapping run did not save it)"
    exit 1
  fi
}

# --- room -------------------------------------------------------------------
run room room_loop 2 1 1 save_map_as:=$ROOM_MAP
run room room_loop 2 2 15
need_map $ROOM_MAP
run room room_loc 2 1 5 slam_mode:=localization slam_map:=$ROOM_MAP
run room room_dropout 2 1 10 "$ROOM_DROPOUT"
run room room_long 10 1 5

# --- warehouse --------------------------------------------------------------
run warehouse warehouse_loop 1 1 1 save_map_as:=$WAREHOUSE_MAP
run warehouse warehouse_loop 1 2 15
need_map $WAREHOUSE_MAP
run warehouse warehouse_loc 1 1 5 slam_mode:=localization slam_map:=$WAREHOUSE_MAP
run warehouse warehouse_dropout 1 1 10 "$WAREHOUSE_DROPOUT" checkpoint_on_scan_lost:=true
run warehouse warehouse_recovery 1 1 5 "$WAREHOUSE_DROPOUT" checkpoint_on_scan_lost:=true \
  operator_recovery:=true operator_mode:=mapping

echo "##### $(date '+%F %T')  series complete"
