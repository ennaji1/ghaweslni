#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${DOCKING_ROBOT_PYTHON:-$ROOT_DIR/.venv-robot/bin/python}"
[[ -x "$PYTHON" ]] || { echo 'Environnement .venv-robot manquant.' >&2;exit 1; }
exec 9>/tmp/ewslni_robot_reel.lock
flock -n 9 || { echo 'Un controleur materiel est deja actif.' >&2;exit 1; }
exec "$PYTHON" "$ROOT_DIR/controle/arduino_direct_node.py" --calibration "$ROOT_DIR/camera_reelle.yaml" "$@"
