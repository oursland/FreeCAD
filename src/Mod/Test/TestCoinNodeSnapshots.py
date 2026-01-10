# SPDX-License-Identifier: LGPL-2.1-or-later

"""
Visual snapshot tests for selected Coin/Inventor nodes.

This test renders a curated set of nodes offscreen and optionally compares the
resulting PNGs against checked-in baselines.

The test is intended to be executed via FreeCAD's test runner:

  FreeCAD -t TestCoinNodeSnapshots

Environment variables:
  - FC_VISUAL_OUT_DIR: output directory (writes actual/expected/diff)
  - FC_VISUAL_BASELINE_DIR: baseline directory (if set, comparisons run)
  - FC_VISUAL_UPDATE_BASELINE: if truthy, overwrite baselines with actual renders
  - FC_VISUAL_WIDTH / FC_VISUAL_HEIGHT: render size (default: 512x512)
  - FC_VISUAL_TOLERANCE: per-channel tolerance (default: 8)
  - FC_VISUAL_MAX_MISMATCH_PCT: mismatch threshold percent (default: 0.20)
  - FC_VISUAL_IGNORE_ALPHA: ignore alpha differences (default: 1)
  - FC_VISUAL_NODES: optional comma-separated node list to run
"""

# This file is intentionally tolerant of missing optional bindings and older
# FreeCAD/Coin/Pivy environments; it is a test harness rather than library code.
# pylint: disable=import-outside-toplevel,broad-exception-caught,deprecated-module
# pylint: disable=too-many-branches,too-many-statements,too-many-locals
# pylint: disable=too-many-return-statements,too-many-arguments,too-many-positional-arguments

import importlib
import os
import sys
import tempfile
import unittest
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

    # Ensure the GUI subsystem is initialized enough for Coin + offscreen GL.
    try:
        FreeCADGui.setupWithoutGUI()
    except Exception:
        # Older setups may not provide this; offscreen rendering can still work.
        pass

    # Ensure built-in modules (e.g. Part/PartGui) are importable even when the test runner
    # didn't pick up the build's Mod/ paths.
    try:
        home = Path(FreeCAD.getHomePath())
        mod_dir = home / "Mod"
        if mod_dir.is_dir():
            for p in (mod_dir, mod_dir / "Part"):
                ps = str(p)
                if ps not in sys.path:
                    sys.path.insert(0, ps)
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


def _make_scene_for_node(coin, type_name: str):
    root = coin.SoSeparator()

    cam = coin.SoOrthographicCamera()
    # Isometric-ish.
    cam.orientation.setValue(coin.SbRotation(-0.353553, -0.146447, -0.353553, -0.853553))
    root.addChild(cam)

    # Light + base color model so nodes render consistently.
    light = coin.SoDirectionalLight()
    root.addChild(light)

    if type_name == "SoDrawingGrid":
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
        root.addChild(_instantiate(coin, "SoDrawingGrid"))
        return root

    if type_name == "SoRegPoint":
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
        return root

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
        label = _instantiate(coin, "SoDatumLabel")
        try:
            label.string.setValue("SoDatumLabel")
            label.textColor.setValue(1.0, 0.45, 0.34)
            label.size.setValue(18)
            label.lineWidth.setValue(2.0)
            label.sampling.setValue(2.0)
            # `createInstance()` returns a generic node proxy (not a typed Python class),
            # so enum constants / helper methods may not be available.
            # `Gui::SoDatumLabel::Type::DISTANCE == 1`.
            label.datumtype.setValue(1)
            label.param1.setValue(0.25)
            label.param2.setValue(0.0)
            label.pnts.setValues(0, 2, [coin.SbVec3f(-0.5, -0.1, 0.0), coin.SbVec3f(0.5, 0.2, 0.0)])
        except Exception:
            pass
        root.addChild(label)
        return root

    if type_name == "SoTextLabel":
        # Ensure the label isn't white-on-white (SoTextLabel background defaults to white and the
        # inherited material may also be white depending on state defaults).
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(0.05, 0.05, 0.05)
        root.addChild(mat)

        label = _instantiate(coin, "SoTextLabel")
        try:
            label.string.setValues(0, 2, ["SoTextLabel", "Coin geometry"])
            label.background.setValue(True)
            label.backgroundColor.setValue(0.95, 0.95, 0.85)
            label.frameSize.setValue(8.0)
        except Exception:
            pass
        root.addChild(label)
        return root

    if type_name == "SoStringLabel":
        label = _instantiate(coin, "SoStringLabel")
        try:
            label.string.setValue("SoStringLabel")
            label.size.setValue(18)
            # Default SoStringLabel textColor is white, which can become invisible on the white
            # snapshot background depending on the GL blending / alpha handling.
            label.textColor.setValue(0.05, 0.05, 0.05)
        except Exception:
            pass
        root.addChild(label)
        return root

    if type_name == "SoFCBackgroundGradient":
        grad = _instantiate(coin, "SoFCBackgroundGradient")
        try:
            if hasattr(grad, "setColorGradient"):
                grad.setColorGradient(coin.SbColor(0.2, 0.2, 0.6), coin.SbColor(0.9, 0.9, 1.0))
        except Exception:
            pass
        root.addChild(grad)
        return root

    if type_name in ("SoNaviCube", "SoNaviCubeTranslucent", "SoNaviCubeHiliteFront"):
        if type_name == "SoNaviCubeTranslucent":
            # Provide visible background so translucency can be verified.
            try:
                grad = _instantiate(coin, "SoFCBackgroundGradient")
                if hasattr(grad, "setColorGradient"):
                    top = coin.SbColor(0.15, 0.15, 0.20)
                    bottom = coin.SbColor(0.45, 0.45, 0.55)
                    grad.setColorGradient(top, bottom)
                root.addChild(grad)
            except Exception:
                pass

        cube = _instantiate(coin, "SoNaviCube")
        try:
            cube.size.setValue(1.0)
            cube.opacity.setValue(0.55 if type_name == "SoNaviCubeTranslucent" else 1.0)
            cube.borderWidth.setValue(0.02)
            cube.showCoordinateSystem.setValue(True)
            cube.cameraIsOrthographic.setValue(True)
            if type_name == "SoNaviCubeHiliteFront":
                # Gui::SoNaviCube::PickId::Front (see `src/Gui/Inventor/SoNaviCube.h`).
                cube.hiliteId.setValue(1)
            width = float(int(os.environ.get("FC_VISUAL_WIDTH", "512")))
            height = float(int(os.environ.get("FC_VISUAL_HEIGHT", "512")))

            # Render like a real overlay: small square in the corner.
            overlay = max(64.0, min(width, height) * 0.60)
            margin = 8.0
            cube.viewportRect.setValue(
                width - overlay - margin,
                height - overlay - margin,
                overlay,
                overlay,
            )

            # Mimic what the controller does: orient the cube from the viewer camera.
            cube.cameraOrientation.setValue(cam.orientation.getValue())
        except Exception:
            pass
        root.addChild(cube)
        return root

    if type_name in (
        "SoBrepEdgeSet",
        "SoBrepEdgeSetHighlight",
        "SoBrepEdgeSetSelection",
        "SoBrepPointSet",
        "SoBrepPointSetHighlight",
        "SoBrepPointSetSelection",
        "SoBrepFaceSet",
        "SoBrepFaceSetHighlight",
        "SoBrepFaceSetSelection",
        "SoFCControlPoints",
    ):
        # These are provided by PartGui, so ensure it is imported to register the Coin types.
        try:
            importlib.import_module("PartGui")
        except Exception:
            pass

    if type_name in (
        "SoPolygon",
        "SoPolygonOpen",
        "SoPolygonStartIndex",
        "SoPolygonNonPlanar",
        "SoFCIndexedFaceSet",
        "SoFCIndexedFaceSetPerFaceColor",
        "SoFCIndexedFaceSetPerVertexColor",
        "SoFCIndexedFaceSetTranslucent",
    ):
        # These are provided by MeshGui, so ensure it is imported to register the Coin types.
        try:
            importlib.import_module("MeshGui")
        except Exception:
            pass

    if type_name in ("SoPolygon", "SoPolygonOpen", "SoPolygonStartIndex", "SoPolygonNonPlanar"):
        coords = coin.SoCoordinate3()
        if type_name == "SoPolygonStartIndex":
            # Same square, but offset in the coordinate array.
            coords.point.setValues(
                0,
                7,
                [
                    coin.SbVec3f(-0.9, -0.2, 0.0),
                    coin.SbVec3f(-0.8, 0.1, 0.0),
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, 0.7, 0.0),
                    coin.SbVec3f(-0.7, 0.7, 0.0),
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                ],
            )
        elif type_name == "SoPolygonNonPlanar":
            coords.point.setValues(
                0,
                5,
                [
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, -0.7, 0.03),
                    coin.SbVec3f(0.7, 0.7, -0.02),
                    coin.SbVec3f(-0.7, 0.7, 0.01),
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                ],
            )
        elif type_name == "SoPolygonOpen":
            coords.point.setValues(
                0,
                4,
                [
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, 0.7, 0.0),
                    coin.SbVec3f(-0.7, 0.7, 0.0),
                ],
            )
        else:
            # Closed loop (last point repeats).
            coords.point.setValues(
                0,
                5,
                [
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, -0.7, 0.0),
                    coin.SbVec3f(0.7, 0.7, 0.0),
                    coin.SbVec3f(-0.7, 0.7, 0.0),
                    coin.SbVec3f(-0.7, -0.7, 0.0),
                ],
            )
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.10, 0.25, 0.80)
        poly = _instantiate(coin, "SoPolygon")
        try:
            if type_name == "SoPolygonStartIndex":
                poly.startIndex.setValue(2)
                poly.numVertices.setValue(5)
            else:
                poly.startIndex.setValue(0)
                poly.numVertices.setValue(coords.point.getNum())
            poly.render.setValue(True)
        except Exception:
            pass
        root.addChild(coords)
        root.addChild(material)
        root.addChild(poly)
        return root

    if type_name in (
        "SoFCIndexedFaceSet",
        "SoFCIndexedFaceSetPerFaceColor",
        "SoFCIndexedFaceSetPerVertexColor",
        "SoFCIndexedFaceSetTranslucent",
    ):
        # SoFCIndexedFaceSet test geometry does not provide normals. Without normals, lighting
        # contributes only ambient, which makes all material variants appear nearly black.
        # Provide one normal per face (triangle) so PER_FACE/PER_VERTEX material bindings
        # visibly differ under the default directional light.
        normals = coin.SoNormal()
        normals.vector.setValues(
            0,
            12,
            [
                coin.SbVec3f(0.0, 0.0, -1.0),
                coin.SbVec3f(0.0, 0.0, -1.0),
                coin.SbVec3f(0.0, 0.0, 1.0),
                coin.SbVec3f(0.0, 0.0, 1.0),
                coin.SbVec3f(0.0, -1.0, 0.0),
                coin.SbVec3f(0.0, -1.0, 0.0),
                coin.SbVec3f(1.0, 0.0, 0.0),
                coin.SbVec3f(1.0, 0.0, 0.0),
                coin.SbVec3f(0.0, 1.0, 0.0),
                coin.SbVec3f(0.0, 1.0, 0.0),
                coin.SbVec3f(-1.0, 0.0, 0.0),
                coin.SbVec3f(-1.0, 0.0, 0.0),
            ],
        )
        normal_bind = coin.SoNormalBinding()
        normal_bind.value = coin.SoNormalBinding.PER_FACE
        root.addChild(normals)
        root.addChild(normal_bind)

        if type_name == "SoFCIndexedFaceSetTranslucent":
            # Provide visible background so translucency can be verified.
            try:
                grad = _instantiate(coin, "SoFCBackgroundGradient")
                if hasattr(grad, "setColorGradient"):
                    top = coin.SbColor(0.15, 0.15, 0.20)
                    bottom = coin.SbColor(0.95, 0.95, 1.0)
                    grad.setColorGradient(top, bottom)
                root.addChild(grad)
            except Exception:
                pass

        # Simple cube: 12 triangles (each terminated by -1).
        coords = coin.SoCoordinate3()
        coords.point.setValues(
            0,
            8,
            [
                coin.SbVec3f(-0.6, -0.6, -0.6),
                coin.SbVec3f(0.6, -0.6, -0.6),
                coin.SbVec3f(0.6, 0.6, -0.6),
                coin.SbVec3f(-0.6, 0.6, -0.6),
                coin.SbVec3f(-0.6, -0.6, 0.6),
                coin.SbVec3f(0.6, -0.6, 0.6),
                coin.SbVec3f(0.6, 0.6, 0.6),
                coin.SbVec3f(-0.6, 0.6, 0.6),
            ],
        )
        faces = _instantiate(coin, "SoFCIndexedFaceSet")
        try:
            faces.coordIndex.setValues(
                0,
                48,
                [
                    0, 1, 2, -1, 0, 2, 3, -1,  # bottom (-Z)
                    4, 6, 5, -1, 4, 7, 6, -1,  # top (+Z)
                    0, 4, 5, -1, 0, 5, 1, -1,  # -Y
                    1, 5, 6, -1, 1, 6, 2, -1,  # +X
                    2, 6, 7, -1, 2, 7, 3, -1,  # +Y
                    3, 7, 4, -1, 3, 4, 0, -1,  # -X
                ],
            )
        except Exception:
            pass
        material = coin.SoMaterial()
        if type_name == "SoFCIndexedFaceSetPerFaceColor":
            # 12 faces (triangles).
            material.diffuseColor.setValues(
                0,
                12,
                [
                    coin.SbColor(0.90, 0.25, 0.25),
                    coin.SbColor(0.90, 0.55, 0.25),
                    coin.SbColor(0.90, 0.80, 0.25),
                    coin.SbColor(0.65, 0.90, 0.25),
                    coin.SbColor(0.25, 0.90, 0.25),
                    coin.SbColor(0.25, 0.90, 0.65),
                    coin.SbColor(0.25, 0.90, 0.90),
                    coin.SbColor(0.25, 0.65, 0.90),
                    coin.SbColor(0.25, 0.25, 0.90),
                    coin.SbColor(0.65, 0.25, 0.90),
                    coin.SbColor(0.90, 0.25, 0.90),
                    coin.SbColor(0.90, 0.25, 0.65),
                ],
            )
            bind = coin.SoMaterialBinding()
            bind.value = coin.SoMaterialBinding.PER_FACE
            root.addChild(bind)
        elif type_name == "SoFCIndexedFaceSetPerVertexColor":
            material.diffuseColor.setValues(
                0,
                8,
                [
                    coin.SbColor(0.95, 0.25, 0.25),
                    coin.SbColor(0.95, 0.75, 0.25),
                    coin.SbColor(0.25, 0.95, 0.25),
                    coin.SbColor(0.25, 0.95, 0.95),
                    coin.SbColor(0.25, 0.25, 0.95),
                    coin.SbColor(0.95, 0.25, 0.95),
                    coin.SbColor(0.70, 0.70, 0.70),
                    coin.SbColor(0.15, 0.15, 0.15),
                ],
            )
            bind = coin.SoMaterialBinding()
            # IndexedFaceSet colors are bound by index; use coordIndex as the color index
            # so each corner uses the corresponding material color.
            bind.value = coin.SoMaterialBinding.PER_VERTEX_INDEXED
            try:
                coord_index_values = faces.coordIndex.getValues(0)
                faces.materialIndex.setValues(
                    0,
                    faces.coordIndex.getNum(),
                    coord_index_values,
                )
            except Exception:
                pass
            root.addChild(bind)
        else:
            material.diffuseColor.setValue(0.70, 0.70, 0.75)
            if type_name == "SoFCIndexedFaceSetTranslucent":
                material.transparency.setValue(0.55)
        root.addChild(coords)
        root.addChild(material)
        root.addChild(faces)
        return root

    if type_name in ("SoBrepEdgeSet", "SoBrepEdgeSetHighlight", "SoBrepEdgeSetSelection"):
        coords = coin.SoCoordinate3()
        coords.point.setValues(
            0,
            5,
            [
                coin.SbVec3f(-0.6, -0.6, 0.0),
                coin.SbVec3f(0.6, -0.6, 0.0),
                coin.SbVec3f(0.6, 0.6, 0.0),
                coin.SbVec3f(-0.6, 0.6, 0.0),
                coin.SbVec3f(0.0, 0.0, 0.0),
            ],
        )
        style = coin.SoDrawStyle()
        style.lineWidth.setValue(3.0)
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.05, 0.05, 0.05)

        edges = _instantiate(coin, "SoBrepEdgeSet")
        # Square + diagonals.
        edges.coordIndex.setValues(0, 14, [0, 1, 2, 3, 0, -1, 0, 2, -1, 1, 3, -1, 4, -1])
        if type_name == "SoBrepEdgeSetHighlight":
            try:
                edges.highlightCoordIndex.setValues(0, 3, [0, 2, -1])
                edges.highlightColor.setValue(1.0, 0.0, 0.0)
            except Exception:
                pass
        elif type_name == "SoBrepEdgeSetSelection":
            try:
                edges.selectionCoordIndex.setValues(0, 3, [1, 3, -1])
                edges.selectionColor.setValue(0.0, 0.6, 0.0)
            except Exception:
                pass

        root.addChild(coords)
        root.addChild(style)
        root.addChild(material)
        root.addChild(edges)
        return root

    if type_name in ("SoBrepPointSet", "SoBrepPointSetHighlight", "SoBrepPointSetSelection"):
        coords = coin.SoCoordinate3()
        coords.point.setValues(
            0,
            9,
            [
                coin.SbVec3f(-0.6, -0.6, 0.0),
                coin.SbVec3f(0.0, -0.6, 0.0),
                coin.SbVec3f(0.6, -0.6, 0.0),
                coin.SbVec3f(-0.6, 0.0, 0.0),
                coin.SbVec3f(0.0, 0.0, 0.0),
                coin.SbVec3f(0.6, 0.0, 0.0),
                coin.SbVec3f(-0.6, 0.6, 0.0),
                coin.SbVec3f(0.0, 0.6, 0.0),
                coin.SbVec3f(0.6, 0.6, 0.0),
            ],
        )
        style = coin.SoDrawStyle()
        style.pointSize.setValue(7.0)
        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.05, 0.05, 0.05)

        pts = _instantiate(coin, "SoBrepPointSet")
        try:
            pts.startIndex.setValue(0)
            pts.numPoints.setValue(-1)
        except Exception:
            pass
        if type_name == "SoBrepPointSetHighlight":
            try:
                pts.highlightCoordIndex.setValues(0, 1, [4])
                pts.highlightColor.setValue(1.0, 0.0, 0.0)
            except Exception:
                pass
        elif type_name == "SoBrepPointSetSelection":
            try:
                pts.selectionCoordIndex.setValues(0, 4, [0, 2, 6, 8])
                pts.selectionColor.setValue(0.0, 0.6, 0.0)
            except Exception:
                pass

        root.addChild(coords)
        root.addChild(style)
        root.addChild(material)
        root.addChild(pts)
        return root

    if type_name in ("SoBrepFaceSet", "SoBrepFaceSetHighlight", "SoBrepFaceSetSelection"):
        # Simple cube: 6 parts (faces), 2 triangles each.
        coords = coin.SoCoordinate3()
        coords.point.setValues(
            0,
            8,
            [
                coin.SbVec3f(-0.5, -0.5, -0.5),  # 0
                coin.SbVec3f(0.5, -0.5, -0.5),  # 1
                coin.SbVec3f(0.5, 0.5, -0.5),  # 2
                coin.SbVec3f(-0.5, 0.5, -0.5),  # 3
                coin.SbVec3f(-0.5, -0.5, 0.5),  # 4
                coin.SbVec3f(0.5, -0.5, 0.5),  # 5
                coin.SbVec3f(0.5, 0.5, 0.5),  # 6
                coin.SbVec3f(-0.5, 0.5, 0.5),  # 7
            ],
        )

        material = coin.SoMaterial()
        material.diffuseColor.setValue(0.75, 0.75, 0.78)

        faces = _instantiate(coin, "SoBrepFaceSet")
        # Each triangle ends with -1; partIndex counts triangles per part.
        faces.coordIndex.setValues(
            0,
            48,
            [
                # Bottom (z=-0.5)
                0,
                1,
                2,
                -1,
                0,
                2,
                3,
                -1,
                # Top (z=+0.5)
                4,
                6,
                5,
                -1,
                4,
                7,
                6,
                -1,
                # Front (y=-0.5)
                0,
                5,
                1,
                -1,
                0,
                4,
                5,
                -1,
                # Back (y=+0.5)
                3,
                2,
                6,
                -1,
                3,
                6,
                7,
                -1,
                # Left (x=-0.5)
                0,
                3,
                7,
                -1,
                0,
                7,
                4,
                -1,
                # Right (x=+0.5)
                1,
                5,
                6,
                -1,
                1,
                6,
                2,
                -1,
            ],
        )
        faces.partIndex.setValues(0, 6, [2, 2, 2, 2, 2, 2])

        if type_name == "SoBrepFaceSetHighlight":
            try:
                # Highlight the front face (part 2).
                faces.highlightPartIndex.setValue(2)
                faces.highlightColor.setValue(1.0, 0.0, 0.0)
            except Exception:
                pass
        elif type_name == "SoBrepFaceSetSelection":
            try:
                # Select a couple of faces to exercise the selection overlay.
                faces.selectionPartIndex.setValues(0, 2, [1, 5])
                faces.selectionColor.setValue(0.0, 0.6, 0.0)
            except Exception:
                pass

        root.addChild(coords)
        root.addChild(material)
        root.addChild(faces)
        return root

    if type_name == "SoFCControlPoints":
        # A small 3x3 pole grid with 2x2 knots appended.
        coords = coin.SoCoordinate3()
        pts = []
        for u in (-0.6, 0.0, 0.6):
            for v in (-0.6, 0.0, 0.6):
                pts.append(coin.SbVec3f(u, v, 0.0))
        for u in (-0.3, 0.3):
            for v in (-0.3, 0.3):
                pts.append(coin.SbVec3f(u, v, 0.15))
        coords.point.setValues(0, len(pts), pts)

        cp = _instantiate(coin, "SoFCControlPoints")
        try:
            cp.numPolesU.setValue(3)
            cp.numPolesV.setValue(3)
            cp.numKnotsU.setValue(2)
            cp.numKnotsV.setValue(2)
        except Exception:
            pass

        root.addChild(coords)
        root.addChild(cp)
        return root

    root.addChild(_instantiate(coin, type_name))
    return root


def _render_png(FreeCADGui, coin, root, out_path: Path, width: int, height: int) -> None:
    viewport = coin.SbViewportRegion(width, height)
    # Tighten the camera to what we're rendering.
    cam = root.getChild(0)
    # Some GUI helper nodes (e.g. SoDatumLabel) compute a camera-dependent bounding box.
    # That makes `SoCamera::viewAll()` unstable and can shift the framing of the snapshot.
    # For these, frame the camera from the rest of the scene and render the full graph.
    removed = None
    try:
        dtype = coin.SoType.fromName("SoDatumLabel")
        if not dtype.isBad():
            search = coin.SoSearchAction()
            search.setType(dtype)
            search.setSearchingAll(False)
            search.apply(root)
            path = search.getPath()
            if path is not None and path.getLength() >= 2:
                label = path.getTail()
                parent = path.getNode(path.getLength() - 2)
                try:
                    idx = parent.findChild(label)
                except Exception:
                    idx = -1
                if idx >= 0:
                    # Keep the node alive while it's detached (Coin ref-counting).
                    try:
                        label.ref()
                    except Exception:
                        pass
                    parent.removeChild(idx)
                    removed = (parent, idx, label)
    except Exception:
        removed = None

    cam.viewAll(root, viewport)

    if removed is not None:
        parent, idx, label = removed
        try:
            parent.insertChild(label, idx)
        except Exception:
            # Best-effort restore: append if insertion isn't supported.
            try:
                parent.addChild(label)
            except Exception:
                pass
        try:
            label.unref()
        except Exception:
            pass
    # `SoCamera::viewAll()` can choose a near plane that clips geometry located near the origin.
    # This shows up particularly with `SoText2`/`SoTextLabel` (text draws, but gets clipped away).
    try:
        cam.nearDistance.setValue(min(cam.nearDistance.getValue(), 0.1))
    except Exception:
        pass

    out_path.parent.mkdir(parents=True, exist_ok=True)
    off = FreeCADGui.SoQtOffscreenRenderer(width, height)
    off.setBackgroundColor(1, 1, 1)
    root.ref()
    off.render(root)
    off.writeToImage(str(out_path))
    root.unref()


def _non_background_pixel_count(path: Path) -> int:
    from PySide2.QtGui import QImage  # type: ignore

    img = QImage(str(path))
    if img.isNull():
        return 0
    white = 0xFFFFFFFF
    count = 0
    for y in range(img.height()):
        for x in range(img.width()):
            if img.pixel(x, y) != white:
                count += 1
    return count


def _compare_images(
    expected_path: Path,
    actual_path: Path,
    diff_path: Path,
    *,
    tolerance: int,
    ignore_alpha: bool,
    max_mismatched_pixels: int,
) -> tuple[bool, str]:
    from PySide2.QtGui import QColor, QImage  # type: ignore

    expected = QImage(str(expected_path))
    actual = QImage(str(actual_path))

    if expected.isNull():
        return False, f"baseline is not a readable image: {expected_path}"
    if actual.isNull():
        return False, f"actual is not a readable image: {actual_path}"

    if expected.size() != actual.size():
        diff_path.parent.mkdir(parents=True, exist_ok=True)
        diff = QImage(actual.size(), QImage.Format_ARGB32)
        diff.fill(QColor(255, 0, 255, 255))
        diff.save(str(diff_path))
        msg = (
            f"size mismatch: expected {expected.width()}x{expected.height()} "
            f"vs actual {actual.width()}x{actual.height()}"
        )
        return (
            False,
            msg,
        )

    expected = expected.convertToFormat(QImage.Format_ARGB32)
    actual = actual.convertToFormat(QImage.Format_ARGB32)

    diff = QImage(actual.size(), QImage.Format_ARGB32)
    mismatched = 0

    for y in range(actual.height()):
        for x in range(actual.width()):
            ep = expected.pixel(x, y)
            ap = actual.pixel(x, y)

            er = (ep >> 16) & 0xFF
            eg = (ep >> 8) & 0xFF
            eb = ep & 0xFF
            ea = (ep >> 24) & 0xFF

            ar = (ap >> 16) & 0xFF
            ag = (ap >> 8) & 0xFF
            ab = ap & 0xFF
            aa = (ap >> 24) & 0xFF

            dr = abs(er - ar)
            dg = abs(eg - ag)
            db = abs(eb - ab)
            da = abs(ea - aa) if not ignore_alpha else 0

            dmax = max(dr, dg, db, da)
            if dmax > tolerance:
                mismatched += 1
                diff.setPixel(x, y, QColor(255, 0, 0, 255).rgba())
            else:
                # Keep a lightly-dimmed version of the actual pixel for context.
                diff.setPixel(x, y, QColor(ar // 2, ag // 2, ab // 2, 255).rgba())

    if mismatched > 0:
        diff_path.parent.mkdir(parents=True, exist_ok=True)
        diff.save(str(diff_path))

    if mismatched > max_mismatched_pixels:
        msg = (
            f"mismatched pixels {mismatched} > {max_mismatched_pixels} "
            f"(tolerance={tolerance}, ignore_alpha={ignore_alpha})"
        )
        return (
            False,
            msg,
        )

    return True, f"mismatched pixels {mismatched} <= {max_mismatched_pixels}"


class CoinNodeSnapshotTestCase(unittest.TestCase):
    """Render Coin nodes offscreen and compare against PNG baselines."""

    def test_coin_node_snapshots(self):
        """Render each configured node and compare against baseline images."""
        _, FreeCADGui, coin = _require_gui()

        nodes_env = os.environ.get("FC_VISUAL_NODES", "")
        if nodes_env.strip():
            node_types = [n.strip() for n in nodes_env.split(",") if n.strip()]
        else:
            node_types = [
                "SoDrawingGrid",
                "SoRegPoint",
                "SoDatumLabel",
                "SoTextLabel",
                "SoStringLabel",
                "SoFCBackgroundGradient",
                "SoNaviCube",
                "SoNaviCubeTranslucent",
                "SoNaviCubeHiliteFront",
                "SoBrepEdgeSet",
                "SoBrepEdgeSetHighlight",
                "SoBrepEdgeSetSelection",
                "SoBrepPointSet",
                "SoBrepPointSetHighlight",
                "SoBrepPointSetSelection",
                "SoBrepFaceSet",
                "SoBrepFaceSetHighlight",
                "SoBrepFaceSetSelection",
                "SoFCControlPoints",
                "SoPolygon",
                "SoPolygonOpen",
                "SoPolygonStartIndex",
                "SoPolygonNonPlanar",
                "SoFCIndexedFaceSet",
                "SoFCIndexedFaceSetPerFaceColor",
                "SoFCIndexedFaceSetPerVertexColor",
                "SoFCIndexedFaceSetTranslucent",
            ]

        width = int(os.environ.get("FC_VISUAL_WIDTH", "512"))
        height = int(os.environ.get("FC_VISUAL_HEIGHT", "512"))

        out_dir = Path(
            os.environ.get(
                "FC_VISUAL_OUT_DIR",
                os.path.join(tempfile.gettempdir(), "FreeCADTesting", "CoinNodeSnapshots"),
            )
        )

        baseline_dir_env = os.environ.get("FC_VISUAL_BASELINE_DIR", "").strip()
        baseline_dir = Path(baseline_dir_env) if baseline_dir_env else None
        update_baseline = os.environ.get("FC_VISUAL_UPDATE_BASELINE", "").strip() not in (
            "",
            "0",
            "false",
            "False",
        )

        tolerance = int(os.environ.get("FC_VISUAL_TOLERANCE", "8"))
        tolerance = max(0, min(tolerance, 255))
        ignore_alpha = os.environ.get("FC_VISUAL_IGNORE_ALPHA", "1").strip() not in (
            "",
            "0",
            "false",
            "False",
        )
        max_mismatch_pct = float(os.environ.get("FC_VISUAL_MAX_MISMATCH_PCT", "0.20"))
        max_mismatch_pct = max(0.0, min(max_mismatch_pct, 100.0))
        max_mismatched_pixels = int((width * height) * (max_mismatch_pct / 100.0))

        for type_name in node_types:
            with self.subTest(node=type_name):
                root = _make_scene_for_node(coin, type_name)
                actual_dir = out_dir / "actual"
                expected_dir = out_dir / "expected"
                diff_dir = out_dir / "diff"

                actual_path = actual_dir / f"{type_name}.png"
                _render_png(FreeCADGui, coin, root, actual_path, width, height)
                self.assertTrue(actual_path.exists(), f"missing snapshot: {actual_path}")
                self.assertGreater(actual_path.stat().st_size, 0, f"empty snapshot: {actual_path}")
                self.assertGreater(
                    _non_background_pixel_count(actual_path),
                    10,
                    f"snapshot seems empty (all background): {actual_path}",
                )

                if baseline_dir is None:
                    continue

                baseline_dir.mkdir(parents=True, exist_ok=True)
                baseline_path = baseline_dir / f"{type_name}.png"

                if update_baseline:
                    baseline_path.write_bytes(actual_path.read_bytes())
                    continue

                if not baseline_path.exists():
                    self.fail(
                        f"missing baseline: {baseline_path} "
                        "(run with FC_VISUAL_UPDATE_BASELINE=1)"
                    )

                expected_path = expected_dir / f"{type_name}.png"
                expected_dir.mkdir(parents=True, exist_ok=True)
                expected_path.write_bytes(baseline_path.read_bytes())

                ok, msg = _compare_images(
                    expected_path,
                    actual_path,
                    diff_dir / f"{type_name}.png",
                    tolerance=tolerance,
                    ignore_alpha=ignore_alpha,
                    max_mismatched_pixels=max_mismatched_pixels,
                )
                self.assertTrue(ok, msg)
