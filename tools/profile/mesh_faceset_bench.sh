#!/usr/bin/env bash
set -euo pipefail

freecadcmd="${1:-./build/clang-mold-debug/bin/FreeCADCmd}"
shift || true

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"

if [ "$#" -eq 0 ]; then
  set -- node=SoFCIndexedFaceSet tris=200000 frames=120 warmup=20 width=1024 height=1024
fi

exec "$freecadcmd" -t TestMeshRenderPerf --pass "$@"
