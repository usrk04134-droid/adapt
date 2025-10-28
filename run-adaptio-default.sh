#!/usr/bin/env bash
set -euo pipefail

# Run Adaptio with default settings.
# Prefer Nix (per README), fallback to local build script.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v nix >/dev/null 2>&1; then
  echo "[adaptio] Running via Nix with default settings..."
  (
    cd "$SCRIPT_DIR/adaptio"
    exec nix run .# -- "$@"
  )
fi

if [ -f "$SCRIPT_DIR/adaptio/adaptio.sh" ]; then
  echo "[adaptio] Nix not found; building and running locally (release) with defaults..."
  bash "$SCRIPT_DIR/adaptio/adaptio.sh" --build --release
  exec bash "$SCRIPT_DIR/adaptio/adaptio.sh" --run --release -- "$@"
fi

echo "Error: Could not locate Nix or adaptio/adaptio.sh. Run this from the repository root."
exit 1
