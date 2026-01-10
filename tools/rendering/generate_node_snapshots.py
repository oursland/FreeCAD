#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later

"""
Generate simple PNG snapshots for selected FreeCAD Coin nodes (GUI-only).

Typical usage (from a FreeCAD GUI build directory):

  ./bin/FreeCAD -c ../FreeCAD/tools/rendering/generate_node_snapshots.py --out /tmp/fc-snaps

Or from inside FreeCAD's Python console:

  exec(open("tools/rendering/generate_node_snapshots.py", "r", encoding="utf-8").read())
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# This is a standalone convenience script intended to be run in different
# FreeCAD/Python environments; it intentionally uses best-effort imports and
# broad error handling for optional APIs.
# pylint: disable=import-outside-toplevel,broad-exception-caught
# pylint: disable=too-many-branches,too-many-statements,too-many-locals


def _require_freecad_gui() -> tuple[object, object, object]:
    try:
        import FreeCAD  # type: ignore
        import FreeCADGui  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(
            "FreeCAD/FreeCADGui modules not available. Run this script via FreeCAD (GUI build)."
        ) from exc

    if not getattr(FreeCAD, "GuiUp", False):  # pragma: no cover
        raise RuntimeError("FreeCAD GUI is not available (FreeCAD.GuiUp is false).")

    try:
        from pivy import coin  # type: ignore
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("pivy.coin is not available; cannot create Coin nodes.") from exc

    return FreeCAD, FreeCADGui, coin


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        default="node-snapshots",
        help="Output directory for PNGs (default: %(default)s)",
    )
    parser.add_argument("--width", type=int, default=800, help="Image width in pixels")
    parser.add_argument("--height", type=int, default=800, help="Image height in pixels")
    parser.add_argument(
        "--background",
        default="Transparent",
        choices=["Transparent", "Current", "White", "Black"],
        help="Background mode for View3D.saveImage() (default: %(default)s)",
    )
    parser.add_argument(
        "--nodes",
        nargs="*",
        default=[
            "SoDrawingGrid",
            "SoRegPoint",
            "SoFCBackgroundGradient",
            "SoDatumLabel",
            "SoTextLabel",
            "SoStringLabel",
            "SoNaviCube",
        ],
        help="Coin node type names to snapshot (default: a small curated set)",
    )
    parser.add_argument(
        "--keep-doc",
        action="store_true",
        help="Do not close the temporary document after rendering",
    )
    return parser.parse_args(argv)


def _create_view(FreeCADGui: object) -> object:
    doc = FreeCADGui.ActiveDocument
    if doc is None:  # pragma: no cover
        raise RuntimeError("No active GUI document.")
    view_obj = doc.createView("Gui::View3DInventor")
    try:
        view_obj.setAnimationEnabled(False)
    except Exception:
        pass
    try:
        view_obj.setCameraType("Orthographic")
    except Exception:
        pass
    try:
        view_obj.viewIsometric()
    except Exception:
        pass
    return view_obj


def _save_active_view(
    FreeCADGui: object,
    out_path: Path,
    width: int,
    height: int,
    background: str,
) -> None:
    FreeCADGui.updateGui()
    view = FreeCADGui.activeDocument().activeView()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    view.saveImage(str(out_path), width, height, background)


def _remove_child(parent: object, child: object) -> None:
    try:
        parent.removeChild(child)
    except Exception:
        pass


def _instantiate(coin: object, type_name: str) -> object:
    t = coin.SoType.fromName(type_name)
    if t.isBad():  # pragma: no cover
        raise RuntimeError(f"Coin type not registered: {type_name}")
    node = t.createInstance()
    if node is None:  # pragma: no cover
        raise RuntimeError(f"Failed to instantiate {type_name}")
    return node


def _make_scene_for_node(
    coin: object,
    type_name: str,
    width: int,
    height: int,
) -> tuple[str, object]:
    root = coin.SoSeparator()

    if type_name == "SoDrawingGrid":
        # Put some 3D geometry behind to make the "draw on top" intent visible.
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.7, 0.7, 0.75)

        cube_trans = coin.SoTranslation()
        cube_trans.translation.setValue(0.0, 0.0, -0.5)

        cube = coin.SoCube()
        cube.width = 1.2
        cube.height = 1.2
        cube.depth = 1.2

        root.addChild(material)
        root.addChild(cube_trans)
        root.addChild(cube)

        grid = _instantiate(coin, "SoDrawingGrid")
        root.addChild(grid)
        return "so_drawing_grid", root

    if type_name == "SoRegPoint":
        # Simple probe (line + 2 points + text) with a background cube for depth context.
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.7, 0.7, 0.75)

        cube_trans = coin.SoTranslation()
        cube_trans.translation.setValue(0.0, 0.0, -0.5)

        cube = coin.SoCube()
        cube.width = 1.2
        cube.height = 1.2
        cube.depth = 1.2

        root.addChild(material)
        root.addChild(cube_trans)
        root.addChild(cube)

        probe = _instantiate(coin, "SoRegPoint")
        try:
            probe.base.setValue(0.0, 0.0, 0.0)
            probe.normal.setValue(0.6, 0.7, 0.4)
            probe.length.setValue(1.2)
            probe.color.setValue(1.0, 0.45, 0.34)
            probe.text.setValue("SoRegPoint")
        except Exception:
            pass
        root.addChild(probe)
        return "so_reg_point", root

    if type_name == "SoDatumLabel":
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.7, 0.7, 0.75)

        cube_trans = coin.SoTranslation()
        cube_trans.translation.setValue(0.0, 0.0, -0.5)

        cube = coin.SoCube()
        cube.width = 1.2
        cube.height = 1.2
        cube.depth = 1.2

        root.addChild(material)
        root.addChild(cube_trans)
        root.addChild(cube)

        label_trans = coin.SoTranslation()
        label_trans.translation.setValue(0.0, 0.0, 0.0)

        label = _instantiate(coin, "SoDatumLabel")
        try:
            label.string.setValue("SoDatumLabel")
            label.textColor.setValue(1.0, 0.45, 0.34)
            label.size.setValue(18)
            label.lineWidth.setValue(2.0)
            label.sampling.setValue(2.0)
            label.datumtype.setValue(label.DISTANCE)
            label.param1.setValue(0.25)
            label.param2.setValue(0.0)
            if hasattr(label, "setPoints"):
                label.setPoints(coin.SbVec3f(-0.5, -0.1, 0.0), coin.SbVec3f(0.5, 0.2, 0.0))
            else:
                label.pnts.setValues(
                    0,
                    2,
                    [
                        coin.SbVec3f(-0.5, -0.1, 0.0),
                        coin.SbVec3f(0.5, 0.2, 0.0),
                    ],
                )
        except Exception:
            pass

        root.addChild(label_trans)
        root.addChild(label)
        return "so_datum_label", root

    if type_name == "SoTextLabel":
        trans = coin.SoTranslation()
        trans.translation.setValue(0.0, 0.0, 0.0)
        label = _instantiate(coin, "SoTextLabel")
        try:
            label.string.setValues(0, 2, ["SoTextLabel", "Coin geometry"])
            label.background.setValue(True)
            label.frameSize.setValue(8.0)
        except Exception:
            pass
        root.addChild(trans)
        root.addChild(label)
        return "so_text_label", root

    if type_name == "SoStringLabel":
        trans = coin.SoTranslation()
        trans.translation.setValue(0.0, 0.0, 0.0)
        label = _instantiate(coin, "SoStringLabel")
        try:
            label.string.setValue("SoStringLabel")
            label.size.setValue(18)
        except Exception:
            pass
        root.addChild(trans)
        root.addChild(label)
        return "so_string_label", root

    if type_name == "SoFCBackgroundGradient":
        grad = _instantiate(coin, "SoFCBackgroundGradient")
        try:
            # Best-effort: methods are not guaranteed to be wrapped.
            if hasattr(grad, "setColorGradient"):
                grad.setColorGradient(coin.SbColor(0.2, 0.2, 0.6), coin.SbColor(0.9, 0.9, 1.0))
        except Exception:
            pass
        root.addChild(grad)
        return "so_fc_background_gradient", root

    if type_name == "SoNaviCube":
        cube = _instantiate(coin, "SoNaviCube")
        try:
            cube.size.setValue(1.0)
            cube.opacity.setValue(1.0)
            cube.borderWidth.setValue(0.02)
            cube.showCoordinateSystem.setValue(True)
            cube.cameraIsOrthographic.setValue(True)
            cube.viewportRect.setValue(0.0, 0.0, float(width), float(height))
        except Exception:
            pass
        root.addChild(cube)
        return "so_navi_cube", root

    # Default: just instantiate and attach.
    node = _instantiate(coin, type_name)
    root.addChild(node)
    safe_name = "".join(ch.lower() if ch.isalnum() else "_" for ch in type_name).strip("_")
    return safe_name, root


def main(argv: list[str]) -> int:
    """Script entry point."""
    args = _parse_args(argv)
    FreeCAD, FreeCADGui, coin = _require_freecad_gui()

    out_dir = Path(args.out).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    doc_name = "NodeSnapshots"
    doc = FreeCAD.newDocument(doc_name)
    try:
        FreeCADGui.activateDocument(doc.Name)
    except Exception:
        pass
    try:
        _ = FreeCADGui.getDocument(doc.Name)
    except Exception:
        pass
    _create_view(FreeCADGui)
    view = FreeCADGui.activeDocument().activeView()
    try:
        view.setCameraType("Orthographic")
        view.viewIsometric()
    except Exception:
        pass

    scene_graph = view.getSceneGraph()
    created = []
    failures = 0

    for type_name in args.nodes:
        scene = None
        try:
            file_stem, scene = _make_scene_for_node(coin, type_name, args.width, args.height)
            scene_graph.addChild(scene)
            try:
                view.fitAll()
            except Exception:
                pass
            out_path = out_dir / f"{file_stem}.png"
            _save_active_view(FreeCADGui, out_path, args.width, args.height, args.background)
            created.append(out_path)
        except Exception as exc:
            failures += 1
            print(f"[snapshot] {type_name}: FAILED: {exc}", file=sys.stderr)
        finally:
            if scene is not None:
                _remove_child(scene_graph, scene)

    print(f"Wrote {len(created)} snapshots to {out_dir}")
    for p in created:
        print(f"- {p}")

    if not args.keep_doc:
        try:
            FreeCAD.closeDocument(doc.Name)
        except Exception:
            pass

    return 1 if failures else 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
