# OpenGL Usage Audit — FreeCAD src/Gui/

Audit date: 2026-03-25
Purpose: Map all direct OpenGL usage for renderer abstraction planning.

## Summary

**22 files with ~735 direct GL call sites** across 7 categories.

## 1. Scene Rendering (4 files, ~134 GL calls, per-frame)

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/Renderer/GLSceneRenderer.cpp` | 42 | Modern VBO/shader path. Our new renderer. |
| `src/Mod/Part/Gui/SoBrepFaceSet.cpp` | 59 | Face tessellation. VBO + immediate mode fallback. |
| `src/Mod/Part/Gui/SoBrepEdgeSet.cpp` | 20 | Edge lines. Immediate mode (glBegin/glEnd). |
| `src/Mod/Part/Gui/SoBrepPointSet.cpp` | 13 | Vertex points. Immediate mode. |

## 2. Selection/Highlighting (2 files, ~5 GL calls, interactive)

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/Selection/SoFCUnifiedSelection.cpp` | 3 | Color blending, depth control via Coin state. |
| `src/Gui/Selection/SoFCSelection.cpp` | 2 | Legacy selection node. |

## 3. Overlay/Annotation (8 files, ~445 GL calls, per-frame)

These are the heaviest GL users outside scene rendering. All use
immediate mode (glBegin/glEnd) and manual matrix management.

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/SoDatumLabel.cpp` | 129 | Constraint/dimension labels. Heaviest annotation file. |
| `src/Gui/GLPainter.cpp` | 97 | 2D painter overlay state management. |
| `src/Gui/NaviCube.cpp` | 85 | Navigation cube. Rendered every frame. |
| `src/Gui/Inventor/SoFCBackgroundGradient.cpp` | 63 | Gradient background. |
| `src/Gui/SoTextLabel.cpp` | 43 | Text label rendering. |
| `src/Gui/Inventor/SoAxisCrossKit.cpp` | 14 | Coordinate axes cross. |
| `src/Gui/Inventor/SoDrawingGrid.cpp` | 13 | Grid lines. |
| `src/Gui/Flag.cpp` | 1 | Flag/marker color setup. |

## 4. Main Viewer / Offscreen (3 files, ~188 GL calls, mixed)

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/View3DInventorViewer.cpp` | 180 | Main viewer. GL state, background, framebuffer. Per-frame + on-demand. |
| `src/Gui/Quarter/QuarterWidget.cpp` | 4 | Context setup, multisample control. Per-frame. |
| `src/Gui/SoFCOffscreenRenderer.cpp` | 4 | Screenshot/thumbnail via glReadPixels. On-demand. |

## 5. Utility / Buffer Management (2 files, ~43 GL calls, init-time)

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/Renderer/ShaderProgram.cpp` | 28 | Shader compilation (glCreateShader, glCompileShader, etc.). |
| `src/Gui/GLBuffer.cpp` | 15 | VBO wrapper via Coin3D glue layer. |

## 6. VR / Specialized (2 files, ~72 GL calls)

| File | GL Calls | Notes |
|------|----------|-------|
| `src/Gui/CoinRiftWidget.cpp` | 51 | Oculus Rift stereo rendering. Framebuffer per eye. |
| `src/Mod/Part/Gui/SoFCShapeObject.cpp` | 21 | B-spline control point/curve visualization. |

## Abstraction Priorities

### Must abstract for multi-backend (Vulkan, Metal, Hydra):

1. **Rendering commands**: glDrawElements, glDrawArrays, glClear, glViewport
2. **Buffer management**: glGenBuffers, glBindBuffer, glBufferData, glDeleteBuffers
3. **Shader programs**: glCreateShader, glCompileShader, glLinkProgram, glUniform*
4. **Framebuffer objects**: glGenFramebuffers, glBindFramebuffer, glReadPixels
5. **State management**: glEnable/glDisable, glDepthFunc, glBlendFunc, glDepthMask

### Must replace (no equivalent in modern APIs):

1. **Immediate mode**: glBegin/glEnd/glVertex/glColor/glNormal (SoDatumLabel, NaviCube, SoAxisCrossKit, SoDrawingGrid, SoTextLabel, SoFCBackgroundGradient, SoBrepEdgeSet, SoBrepPointSet)
2. **Fixed-function matrix stack**: glMatrixMode, glLoadIdentity, glPushMatrix, glPopMatrix, glOrtho (all annotation files)
3. **Fixed-function lighting**: glLight*, glMaterial* (via Coin's SoGLLazyElement)
4. **Attribute stack**: glPushAttrib/glPopAttrib (GLPainter, View3DInventorViewer)

### Performance note — rotation delay source:

The annotation/overlay rendering path in View3DInventorViewer.cpp (lines 2631-2641)
calls `glra->apply()` for delayed annotations, which triggers Coin's full
SoGLRenderAction traversal. This causes 1-3 second delays when Coin renders
the scene through its legacy path for annotations. The fix is to move
overlay rendering to the SceneRenderer interface.
