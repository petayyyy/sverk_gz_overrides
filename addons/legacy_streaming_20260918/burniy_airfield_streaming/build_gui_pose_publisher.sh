#!/usr/bin/env bash
set -Eeuo pipefail

STREAM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$STREAM_DIR/plugins"
SOURCE="$STREAM_DIR/BurniyGuiPosePublisher.cc"
QRC="$STREAM_DIR/BurniyGuiPosePublisher.qrc"
QRC_SOURCE="$OUTPUT_DIR/qrc_BurniyGuiPosePublisher.cc"
OUTPUT="$OUTPUT_DIR/libBurniyGuiPosePublisher.so"

command -v g++ >/dev/null || { echo "g++ is required to build the GUI pose publisher." >&2; exit 2; }
command -v pkg-config >/dev/null || { echo "pkg-config is required to build the GUI pose publisher." >&2; exit 2; }
command -v rcc >/dev/null || { echo "Qt rcc is required to build the GUI pose publisher." >&2; exit 2; }
[[ -f "$SOURCE" ]] || { echo "Missing source: $SOURCE" >&2; exit 2; }
[[ -f "$QRC" ]] || { echo "Missing resource file: $QRC" >&2; exit 2; }
mkdir -p "$OUTPUT_DIR"

rcc -name BurniyGuiPosePublisher "$QRC" -o "$QRC_SOURCE"
g++ -std=c++17 -O2 -fPIC -shared -Wall -Wextra -Wpedantic \
  "$SOURCE" "$QRC_SOURCE" -o "$OUTPUT" \
  $(pkg-config --cflags --libs gz-gui8 gz-rendering8 gz-transport13 gz-msgs10 gz-plugin2)

echo "Built $OUTPUT"
