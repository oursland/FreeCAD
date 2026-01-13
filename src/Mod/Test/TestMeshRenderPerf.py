# SPDX-License-Identifier: LGPL-2.1-or-later

import json
import math
import os
import sys
import time
import unittest
from dataclasses import dataclass
from pathlib import Path


def _require_gui():
    try:
        import FreeCAD  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("FreeCAD module not available") from exc

    try:
        import FreeCADGui  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise unittest.SkipTest("FreeCADGui not available in this build") from exc

    try:
        from pivy import coin  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise unittest.SkipTest("pivy.coin not available") from exc

    try:
        FreeCADGui.setupWithoutGUI()
    except Exception:
        pass

    # Ensure a QGuiApplication exists so an OpenGL context can be created.
    try:
        from PySide2 import QtGui  # type: ignore

        if QtGui.QGuiApplication.instance() is None:
            QtGui.QGuiApplication(sys.argv)
    except Exception as exc:  # pragma: no cover
        raise unittest.SkipTest("Qt (PySide2) not available") from exc

    return FreeCAD, FreeCADGui, coin


def _instantiate(coin, type_name: str):
    t = coin.SoType.fromName(type_name)
    if t.isBad():
        raise unittest.SkipTest(f"Coin type not registered: {type_name}")
    node = t.createInstance()
    if node is None:
        raise RuntimeError(f"Failed to instantiate {type_name}")
    return node


def _percentile(samples, p: float) -> float:
    if not samples:
        return float("nan")
    if p <= 0:
        return min(samples)
    if p >= 100:
        return max(samples)
    xs = sorted(samples)
    k = (len(xs) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return float(xs[int(k)])
    d0 = xs[f] * (c - k)
    d1 = xs[c] * (k - f)
    return float(d0 + d1)


def _rotate_camera(coin, yaw: float, pitch: float):
    ry = coin.SbRotation(coin.SbVec3f(0, 1, 0), yaw)
    rx = coin.SbRotation(coin.SbVec3f(1, 0, 0), pitch)
    return ry * rx


@dataclass(frozen=True)
class BenchArgs:
    tris: int = 200_000
    frames: int = 120
    warmup: int = 20
    width: int = 1024
    height: int = 1024
    node: str = "SoFCIndexedFaceSet"
    out: str = ""
    write_every: int = 0


def _parse_args(argv) -> BenchArgs:
    # Args are passed via FreeCADCmd's `--pass` mechanism.
    #
    # IMPORTANT: arguments after `--pass` must NOT start with `--`, otherwise
    # FreeCAD's option parser will interpret them as program options.
    #
    # Use `key=value`, e.g.:
    #   --pass node=SoFCIndexedFaceSet tris=200000 frames=120 warmup=20
    kv = {}
    for tok in argv:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        kv[k.strip().lstrip("-")] = v.strip()

    def _get_int(key: str, default: int) -> int:
        if key not in kv:
            return default
        try:
            return int(kv[key])
        except Exception as exc:
            raise ValueError(f"Invalid {key} value: {kv[key]!r}") from exc

    def _get_str(key: str, default: str) -> str:
        return kv.get(key, default)

    return BenchArgs(
        tris=_get_int("tris", 200_000),
        frames=_get_int("frames", 120),
        warmup=_get_int("warmup", 20),
        width=_get_int("width", 1024),
        height=_get_int("height", 1024),
        node=_get_str("node", "SoFCIndexedFaceSet"),
        out=_get_str("out", ""),
        write_every=_get_int("write_every", 0),
    )


def _build_grid_mesh(coin, tris_target: int):
    # Build an N×N grid; each quad produces 2 triangles.
    n = int(math.floor(math.sqrt(max(tris_target, 2) / 2.0))) + 1
    n = max(2, min(n, 4096))
    tris = 2 * (n - 1) * (n - 1)

    coords = coin.SoCoordinate3()
    pts = []
    # Centered in [-1, 1] with a tiny height ripple so lighting isn't uniform.
    for j in range(n):
        y = (j / (n - 1)) * 2.0 - 1.0
        for i in range(n):
            x = (i / (n - 1)) * 2.0 - 1.0
            z = 0.02 * math.sin(x * 6.0) * math.cos(y * 6.0)
            pts.append(coin.SbVec3f(x, y, z))
    coords.point.setValues(0, len(pts), pts)

    indices = []
    for j in range(n - 1):
        row0 = j * n
        row1 = (j + 1) * n
        for i in range(n - 1):
            a = row0 + i
            b = row0 + i + 1
            c = row1 + i + 1
            d = row1 + i
            indices.extend([a, b, c, -1, a, c, d, -1])

    return coords, indices, tris, n


def _make_scene(coin, node_type: str, tris_target: int):
    root = coin.SoSeparator()

    cam = coin.SoPerspectiveCamera()
    cam.heightAngle = 0.70
    root.addChild(cam)

    light = coin.SoDirectionalLight()
    root.addChild(light)

    mat = coin.SoMaterial()
    mat.diffuseColor.setValue(0.70, 0.70, 0.75)
    root.addChild(mat)

    coords, coord_index, tris, n = _build_grid_mesh(coin, tris_target)
    root.addChild(coords)

    if node_type != "SoIndexedFaceSet":
        # Ensure MeshGui types are registered (e.g. SoFCIndexedFaceSet).
        try:
            import MeshGui  # type: ignore  # noqa: F401
        except Exception:
            pass

    faces = _instantiate(coin, node_type) if node_type != "SoIndexedFaceSet" else coin.SoIndexedFaceSet()
    faces.coordIndex.setValues(0, len(coord_index), coord_index)
    root.addChild(faces)

    return root, cam, tris, n


class MeshRenderPerfTestCase(unittest.TestCase):
    """
    Offscreen rendering benchmark for Mesh GUI nodes.

    Run like:
      QT_QPA_PLATFORM=offscreen ./build/<cfg>/bin/FreeCADCmd -t TestMeshRenderPerf --pass \
        node=SoFCIndexedFaceSet tris=200000 frames=120 warmup=20 width=1024 height=1024

    Optional:
      out=/tmp/mesh_perf.json
      write_every=N   (very expensive; forces writeToImage every N frames)
    """

    def test_mesh_render_perf(self):
        FreeCAD, FreeCADGui, coin = _require_gui()

        if "--pass" not in sys.argv:
            raise unittest.SkipTest("Provide args via --pass (see docstring).")
        args = _parse_args(sys.argv[sys.argv.index("--pass") + 1 :])

        root, cam, tris, n = _make_scene(coin, args.node, args.tris)

        viewport = coin.SbViewportRegion(args.width, args.height)
        cam.viewAll(root, viewport)

        off = FreeCADGui.SoQtOffscreenRenderer(args.width, args.height)
        off.setBackgroundColor(1, 1, 1)

        tmp_out = None
        if args.write_every > 0:
            tmp_out = Path(
                os.environ.get("FC_MESH_PERF_TMP_IMAGE", os.path.join("/tmp", "FreeCADMeshPerf.ppm"))
            )

        # Warm up to avoid first-frame shader/driver costs.
        root.ref()
        for i in range(max(0, args.warmup)):
            cam.orientation = _rotate_camera(coin, yaw=i * 0.01, pitch=i * 0.007)
            off.render(root)
            if tmp_out and (args.write_every == 1 or ((i + 1) % args.write_every) == 0):
                off.writeToImage(str(tmp_out))

        times_ms = []
        for i in range(max(1, args.frames)):
            cam.orientation = _rotate_camera(coin, yaw=(i + args.warmup) * 0.01, pitch=(i + args.warmup) * 0.007)
            t0 = time.perf_counter()
            off.render(root)
            t1 = time.perf_counter()
            if tmp_out and (args.write_every == 1 or ((i + 1) % args.write_every) == 0):
                off.writeToImage(str(tmp_out))
            times_ms.append((t1 - t0) * 1000.0)
        root.unref()

        mean_ms = sum(times_ms) / len(times_ms)
        result = {
            "node": args.node,
            "tris_target": args.tris,
            "tris_actual": tris,
            "grid_n": n,
            "frames": args.frames,
            "warmup": args.warmup,
            "width": args.width,
            "height": args.height,
            "write_every": args.write_every,
            "mean_ms": mean_ms,
            "p50_ms": _percentile(times_ms, 50.0),
            "p95_ms": _percentile(times_ms, 95.0),
            "min_ms": min(times_ms),
            "max_ms": max(times_ms),
            "fps_mean": (1000.0 / mean_ms) if mean_ms > 0 else 0.0,
            "freecad_version": FreeCAD.Version(),
        }

        payload = json.dumps(result, indent=2, sort_keys=True)
        if args.out:
            out_path = Path(args.out)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(payload, encoding="utf-8")
        else:
            # Keep stdout as machine-readable JSON by default.
            print(payload)
