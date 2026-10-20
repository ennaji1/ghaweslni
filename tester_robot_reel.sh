#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
exec 9>/tmp/ewslni_robot_reel.lock
flock -n 9 || { echo 'Un essai matériel est déjà actif.' >&2; exit 1; }
PYTHON="$ROOT_DIR/.venv-robot/bin/python"
[[ -x "$PYTHON" ]] || { echo 'Installer .venv-robot selon README.md'; exit 1; }
exec "$PYTHON" "$ROOT_DIR/controle/real_distilled_node.py" "$@"
