#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later

"""
Manage baselines for the Coin node visual snapshot test.

This script runs the existing FreeCAD unittest `TestCoinNodeSnapshots` via FreeCADCmd,
configuring it through environment variables (so we don't depend on FreeCAD forwarding
CLI args to Python).

Examples:

  # Update baselines in-tree (recommended: do this on a controlled setup)
  tools/rendering/manage_coin_node_baselines.py update \
    --baseline-dir tests/visual/baselines/coin-nodes \
    --freecadcmd build/clang-mold-debug/bin/FreeCADCmd

  # Compare current renders against baselines (writes actual/expected/diff under --out-dir)
  tools/rendering/manage_coin_node_baselines.py compare \
    --baseline-dir tests/visual/baselines/coin-nodes \
    --out-dir /tmp/FreeCADTesting/CoinNodeSnapshots \
    --freecadcmd build/clang-mold-debug/bin/FreeCADCmd
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

# pylint: disable=broad-exception-caught,duplicate-code

def _default_freecadcmd() -> str | None:
    candidates = [
        Path("build/clang-mold-debug/bin/FreeCADCmd"),
        Path("build/clang-mold-release/bin/FreeCADCmd"),
    ]
    for p in candidates:
        if p.is_file():
            return str(p)

    # Fall back to first build/*/bin/FreeCADCmd
    for p in sorted(Path("build").glob("*/bin/FreeCADCmd")):
        if p.is_file():
            return str(p)

    return None


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--freecadcmd",
        default=_default_freecadcmd(),
        help="Path to FreeCADCmd (default: auto-detect under build/*/bin/FreeCADCmd)",
    )
    common.add_argument(
        "--qt-platform",
        default="offscreen",
        help="Value for QT_QPA_PLATFORM (default: %(default)s)",
    )
    common.add_argument(
        "--baseline-dir",
        required=True,
        help="Baseline directory containing/writing *.png files",
    )
    common.add_argument(
        "--out-dir",
        default=os.path.join("/tmp", "FreeCADTesting", "CoinNodeSnapshots"),
        help="Artifact output directory (default: %(default)s)",
    )
    common.add_argument("--nodes", default="", help="Comma-separated node type list (optional)")
    common.add_argument("--width", type=int, default=512, help="Image width (default: %(default)s)")
    common.add_argument(
        "--height",
        type=int,
        default=512,
        help="Image height (default: %(default)s)",
    )
    common.add_argument(
        "--tolerance",
        type=int,
        default=8,
        help="Per-channel tolerance (default: %(default)s)",
    )
    common.add_argument(
        "--max-mismatch-pct",
        type=float,
        default=0.20,
        help="Allowed mismatch percent (default: %(default)s)",
    )
    common.add_argument(
        "--ignore-alpha",
        default="1",
        choices=["0", "1"],
        help="Ignore alpha channel (default: %(default)s)",
    )

    sub.add_parser("update", parents=[common], help="Write/update baselines")
    sub.add_parser("compare", parents=[common], help="Compare against baselines")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    """Entry point."""
    args = _parse_args(argv)

    if not args.freecadcmd:
        print(
            "ERROR: could not auto-detect FreeCADCmd; pass --freecadcmd "
            "build/<preset>/bin/FreeCADCmd",
            file=sys.stderr,
        )
        return 2

    freecadcmd = Path(args.freecadcmd)
    if not freecadcmd.is_file():
        print(f"ERROR: FreeCADCmd not found: {freecadcmd}", file=sys.stderr)
        return 2

    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = args.qt_platform
    env["FC_VISUAL_BASELINE_DIR"] = str(Path(args.baseline_dir).resolve())
    env["FC_VISUAL_OUT_DIR"] = str(Path(args.out_dir).resolve())
    env["FC_VISUAL_WIDTH"] = str(int(args.width))
    env["FC_VISUAL_HEIGHT"] = str(int(args.height))
    env["FC_VISUAL_TOLERANCE"] = str(int(args.tolerance))
    env["FC_VISUAL_MAX_MISMATCH_PCT"] = str(float(args.max_mismatch_pct))
    env["FC_VISUAL_IGNORE_ALPHA"] = args.ignore_alpha
    if args.nodes.strip():
        env["FC_VISUAL_NODES"] = args.nodes

    if args.mode == "update":
        env["FC_VISUAL_UPDATE_BASELINE"] = "1"
        cmd = [str(freecadcmd), "-t", "TestCoinNodeSnapshots"]
    else:
        env.pop("FC_VISUAL_UPDATE_BASELINE", None)
        cmd = [str(freecadcmd), "-t", "TestCoinNodeSnapshots"]

    print(f"Running: {' '.join(cmd)}")
    print(f"  QT_QPA_PLATFORM={env['QT_QPA_PLATFORM']}")
    print(f"  FC_VISUAL_BASELINE_DIR={env['FC_VISUAL_BASELINE_DIR']}")
    print(f"  FC_VISUAL_OUT_DIR={env['FC_VISUAL_OUT_DIR']}")
    if "FC_VISUAL_NODES" in env:
        print(f"  FC_VISUAL_NODES={env['FC_VISUAL_NODES']}")

    proc = subprocess.run(cmd, env=env, cwd=str(Path.cwd()), check=False)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
