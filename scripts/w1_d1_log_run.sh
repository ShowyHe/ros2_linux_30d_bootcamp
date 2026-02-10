#!/usr/bin/env bash
set -u
set -o pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <command...>" >&2
  echo "Example: $0 ls -la" >&2
  exit 2
fi

mkdir -p logs
ts="$(date '+%Y%m%d_%H%M%S')"
log="logs/w1_d2_${ts}.log"

echo "[INFO] $(date '+%F %T') cmd: $*" | tee -a "$log"

"$@" 2>&1 | tee -a "$log"
rc=${PIPESTATUS[0]}

echo "[INFO] $(date '+%F %T') exit_code: $rc" | tee -a "$log"
exit "$rc"
