#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Building ch341_can kernel module..."
make -C "$SCRIPT_DIR/src" "$@"