#!/usr/bin/env bash
# update_overrides.sh — установка/обновление SITL-артефактов sverk_gz_overrides.
#
# Раскладывает содержимое репозитория по местам, где его ждёт SITL-стек:
#   models/          -> $PX4_DIR/Tools/simulation/gz/models   (GZ_SIM_RESOURCE_PATH)
#   worlds/*.sdf     -> $PX4_DIR/Tools/simulation/gz/worlds
#   ros_nodes/<pkg>/ -> $ROS_NODES_DIR/<pkg>                  (colcon-пакеты симуляции)
#   px4/*.patch       -> $PX4_DIR                             (настройки PX4, например DDS-топики)
#
# Источник берётся так (по убыванию приоритета):
#   1. --src <dir>        — готовый checkout (так вызывает Dockerfile.sitl);
#   2. checkout, в котором лежит сам скрипт (запуск из клона репозитория);
#   3. свежий git clone --depth 1 --branch $REF $REPO (запуск "ниоткуда",
#      например внутри работающего контейнера sverk_sitl).
#
# Флаги / переменные окружения:
#   --repo URL        | GZ_OVERRIDES_REPO  (default: github.com/petayyyy/sverk_gz_overrides)
#   --ref  BRANCH/TAG | GZ_OVERRIDES_REF   (default: main)
#   --px4-dir DIR     | PX4_DIR            (default: /home/sverk/PX4-Autopilot)
#   --ros-nodes-dir D | ROS_NODES_DIR      (default: /home/sverk/sverk_ws/src/sverk_drone/simulation)
#
# После обновления ros_nodes нужно пересобрать workspace:
#   cd ~/sverk_ws && colcon build --packages-up-to <pkg> && source install/setup.bash
set -euo pipefail

REPO=${GZ_OVERRIDES_REPO:-https://github.com/petayyyy/sverk_gz_overrides.git}
REF=${GZ_OVERRIDES_REF:-main}
PX4_DIR=${PX4_DIR:-/home/sverk/PX4-Autopilot}
ROS_NODES_DIR=${ROS_NODES_DIR:-/home/sverk/sverk_ws/src/sverk_drone/simulation}
SRC=""

usage() { sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --src)           SRC=$2; shift 2 ;;
    --repo)          REPO=$2; shift 2 ;;
    --ref)           REF=$2; shift 2 ;;
    --px4-dir)       PX4_DIR=$2; shift 2 ;;
    --ros-nodes-dir) ROS_NODES_DIR=$2; shift 2 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 1 ;;
  esac
done

# Запуск из checkout'а: рядом со скриптом (на уровень выше) есть models/ и worlds/.
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
if [ -z "$SRC" ] && [ -d "$SCRIPT_DIR/../models" ] && [ -d "$SCRIPT_DIR/../worlds" ]; then
  SRC=$(cd "$SCRIPT_DIR/.." && pwd)
fi

CLEANUP=""
if [ -z "$SRC" ]; then
  SRC=$(mktemp -d "${TMPDIR:-/tmp}/sverk_gz_overrides.XXXXXX")
  CLEANUP=$SRC
  echo ">> cloning $REPO ($REF)"
  git clone --depth 1 --branch "$REF" "$REPO" "$SRC"
fi

[ -d "$SRC/models" ] && [ -d "$SRC/worlds" ] || {
  echo "error: $SRC does not look like a sverk_gz_overrides checkout (no models/ or worlds/)" >&2
  exit 1
}
git -C "$SRC" log -1 --format='>> overrides tip: %H %s' 2>/dev/null || true

# --- models + worlds -> дерево PX4 -------------------------------------------
mkdir -p "$PX4_DIR/Tools/simulation/gz/models" "$PX4_DIR/Tools/simulation/gz/worlds"
cp -r "$SRC/models/." "$PX4_DIR/Tools/simulation/gz/models/"
cp "$SRC"/worlds/*.sdf "$PX4_DIR/Tools/simulation/gz/worlds/"
echo ">> models/worlds -> $PX4_DIR/Tools/simulation/gz"

# --- PX4 settings -------------------------------------------------------------
# The patch is deliberately kept small rather than copying PX4's complete DDS
# profile: this makes the override reviewable and avoids overwriting unrelated
# upstream additions. Each endpoint patch is independently idempotent so an
# existing PX4 checkout can be upgraded from the older outputs-only setup.
DDS_OUTPUTS_PATCH="$SRC/px4/uxrce_dds_topics.patch"
DDS_INPUT_PATCH="$SRC/px4/distance_sensor_input.patch"
OPTICAL_FLOW_INPUT_PATCH="$SRC/px4/optical_flow_input.patch"
DDS_TOPICS="$PX4_DIR/src/modules/uxrce_dds_client/dds_topics.yaml"
if [ -f "$DDS_OUTPUTS_PATCH" ] || [ -f "$DDS_INPUT_PATCH" ] \
  || [ -f "$OPTICAL_FLOW_INPUT_PATCH" ]; then
  if [ ! -f "$DDS_TOPICS" ]; then
    echo "error: PX4 DDS profile not found: $DDS_TOPICS" >&2
    exit 1
  fi

  if ! grep -Fq '/fmu/out/distance_sensor' "$DDS_TOPICS" \
    || ! grep -Fq '/fmu/out/vehicle_optical_flow_vel' "$DDS_TOPICS"; then
    patch --batch --forward --directory="$PX4_DIR" -p1 < "$DDS_OUTPUTS_PATCH"
    echo ">> PX4 DDS sensor outputs configured"
  fi

  if ! grep -Fq '/fmu/in/distance_sensor' "$DDS_TOPICS"; then
    patch --batch --forward --directory="$PX4_DIR" -p1 < "$DDS_INPUT_PATCH"
    echo ">> PX4 DDS distance sensor input configured"
  fi

  if ! grep -Eq '^[[:space:]]*-[[:space:]]+topic:[[:space:]]+/fmu/in/sensor_optical_flow([[:space:]]|$)' "$DDS_TOPICS"; then
    patch --batch --forward --directory="$PX4_DIR" -p1 < "$OPTICAL_FLOW_INPUT_PATCH"
    echo ">> PX4 DDS optical-flow input configured"
  fi
fi

# --- ros_nodes -> workspace (каждый каталог = colcon-пакет) -------------------
installed=0
if [ -d "$SRC/ros_nodes" ]; then
  for pkg in "$SRC"/ros_nodes/*/; do
    [ -d "$pkg" ] || continue
    name=$(basename "$pkg")
    mkdir -p "$ROS_NODES_DIR"
    rm -rf "${ROS_NODES_DIR:?}/${name:?}"
    cp -r "$pkg" "$ROS_NODES_DIR/$name"
    echo ">> ros package: $name -> $ROS_NODES_DIR/$name"
    installed=$((installed + 1))
  done
fi
if [ "$installed" -gt 0 ]; then
  echo ">> $installed package(s) installed; rebuild: colcon build --packages-up-to <pkg>"
else
  echo ">> no ros_nodes packages, skipped"
fi

[ -n "$CLEANUP" ] && rm -rf "$CLEANUP"
echo ">> done"
