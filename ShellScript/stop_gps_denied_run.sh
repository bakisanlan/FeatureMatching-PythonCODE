#!/usr/bin/env bash
set -euo pipefail

# Stops the processes started by run_gps_denied_with_unified_log.sh
# It reads PIDs from .gps_denied_run.pids and tries:
#   SIGINT -> SIGTERM -> SIGKILL

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDFILE="${ROOT_DIR}/.gps_denied_run.pids"

if [[ ! -f "${PIDFILE}" ]]; then
  echo "No PID file found at: ${PIDFILE}"
  echo "Nothing to stop."
  exit 1
fi

echo "Stopping processes listed in ${PIDFILE} ..."

do_kill_pass() {
  local sig="$1"
  while read -r pid; do
    [[ -z "${pid}" ]] && continue
    if kill -0 "${pid}" 2>/dev/null; then
      echo "Sending SIG${sig} to PID ${pid}"
      # If the PID is a process-group leader, also signal the group (-PID)
      kill "-${sig}" "${pid}" 2>/dev/null || true
      kill "-${sig}" "-${pid}" 2>/dev/null || true
    else
      echo "PID ${pid} not running (stale)"
    fi
  done < "${PIDFILE}"
}

do_kill_pass INT
sleep 1

do_kill_pass TERM
sleep 1

do_kill_pass KILL

rm -f "${PIDFILE}"
echo "Stopped. PID file removed."
