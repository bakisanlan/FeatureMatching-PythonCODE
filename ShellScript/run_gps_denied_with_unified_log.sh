#!/usr/bin/env bash
set -euo pipefail

# Run GPS-denied SITL scripts with complete terminal logging via tee.
#
# Creates one log file with ALL terminal output (real-time capture):
#   ${LOG_FILE} - Complete terminal output including ROS2 logs
#
# Usage:
#   ./run_gps_denied_with_unified_log.sh vio
#   ./run_gps_denied_with_unified_log.sh pf
#   ./run_gps_denied_with_unified_log.sh vio DEBUG
#   ./run_gps_denied_with_unified_log.sh pf DEBUG
#   ./run_gps_denied_with_unified_log.sh pf DEBUG 1 INFO
#   ./run_gps_denied_with_unified_log.sh pf DEBUG 0 DEBUG   # allow noisy 3rd-party debug
#
# Positional args:
#   $1 MODE                     : vio | pf
#   $2 LOG_LEVEL                : INFO | DEBUG | WARNING | ERROR (default: INFO)
#   $3 QUIET_THIRD_PARTY_LOGS   : 1 to suppress noisy libs in DEBUG, 0 to disable (default: 1)
#   $4 THIRD_PARTY_LOG_LEVEL    : INFO | WARNING | ERROR (default: INFO)
#
# - "vio": runs OV/test_sim_VIOodom_squareSITL_autotakeoff_landing.py
# - "pf" : runs OV/test_sim_VIOPFodom_squareSITL_autotakeoff_landing.py + ParticleFilterNode.py

MODE="${1:-vio}"
LOG_LEVEL_ARG="${2:-INFO}"
QUIET_THIRD_PARTY_LOGS_ARG="${3:-1}"
THIRD_PARTY_LOG_LEVEL_ARG="${4:-INFO}"

TS="${FEATUREMATCH_RUN_TS:-$(date +%Y%m%d_%H%M%S)}"
export FEATUREMATCH_RUN_TS="${TS}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/logs/${TS}"
mkdir -p "${LOG_DIR}"

PIDFILE="${ROOT_DIR}/.gps_denied_run.pids"
LOG_FILE="${LOG_DIR}/terminal_${MODE}.log"

export LOG_LEVEL="${LOG_LEVEL_ARG}"
export QUIET_THIRD_PARTY_LOGS="${QUIET_THIRD_PARTY_LOGS_ARG}"
export THIRD_PARTY_LOG_LEVEL="${THIRD_PARTY_LOG_LEVEL_ARG}"

# Keep ANSI colors in terminal even though we pipe everything through `tee`.
# setup_unified_logging() uses NO_COLOR=1 to disable coloring.
# You can still set FEATUREMATCH_LOG_NO_COLOR=1 for convenience.
if [[ "${FEATUREMATCH_LOG_NO_COLOR:-0}" == "1" ]]; then
  export NO_COLOR=1
else
  export NO_COLOR="${NO_COLOR:-0}"
fi

echo "Terminal log file: ${LOG_FILE}"
echo "Log level: ${LOG_LEVEL}"
echo "Quiet 3rd-party logs: ${QUIET_THIRD_PARTY_LOGS} (level=${THIRD_PARTY_LOG_LEVEL})"

# Use exec to redirect all output through tee for complete terminal capture
exec > >(tee -a "${LOG_FILE}") 2>&1

echo "Starting main controller (${MODE})..."
if [[ "${MODE}" == "vio" ]]; then
  python "${ROOT_DIR}/OV/test_sim_VIOodom_squareSITL_autotakeoff_landing.py" &
  MAIN_PID=$!
elif [[ "${MODE}" == "pf" ]]; then
  python "${ROOT_DIR}/OV/test_sim_VIOPFodom_squareSITL_autotakeoff_landing.py" &
  MAIN_PID=$!

  # Give the controller a moment to start up before launching PF node
  sleep 1

  echo "Starting ParticleFilterNode..."
  python "${ROOT_DIR}/ParticleFilterNode.py" &
  PF_PID=$!
else
  echo "Unknown mode: ${MODE} (expected: vio or pf)" >&2
  exit 2
fi

# Record PIDs so we can stop the run from another terminal.
: > "${PIDFILE}"
echo "${MAIN_PID}" >> "${PIDFILE}"
if [[ "${MODE}" == "pf" ]]; then
  echo "${PF_PID}" >> "${PIDFILE}"
fi

# NOTE:
# We intentionally keep ${PIDFILE} so you can stop the run from another terminal
# using ./stop_gps_denied_run.sh (e.g., via alias). The stop script removes the pidfile.
# If you want the pidfile removed automatically on normal completion, we do it at the end.

# Wait for the main script and mirror its exit code.
wait "${MAIN_PID}"
MAIN_STATUS=$?

# If PF was started, wait for it too.
if [[ "${MODE}" == "pf" ]]; then
  wait "${PF_PID}" || true
fi

echo "Done. Log saved to: ${LOG_FILE}"
rm -f "${PIDFILE}" || true
exit "${MAIN_STATUS}"
