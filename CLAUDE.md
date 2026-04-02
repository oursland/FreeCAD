# FreeCAD Unified Renderer

## Overview

This branch implements a modern rendering backend for FreeCAD through Coin3D, replacing the legacy OpenGL fixed-function pipeline with an abstract renderer interface. The work is based on [tritao's modern renderer design](https://github.com/tritao/coin/blob/8d9d7f6dcc19225735bd10e67c14318c54eb7fae/docs/modern-renderer-design.md).

## Architecture

### Coin Side (unified-renderer branch in coin repo)

- **SoModernRenderAction**: Traverses the scene graph, builds an API-agnostic intermediate representation (IR) of draw commands
- **SoDrawList**: Ordered list of SoRenderCommand structs — complete draw call descriptions with geometry, material, state, pick data, selection data
- **SoRenderBackend**: Abstract interface for GPU backends (OpenGL, Vulkan, Metal, Hydra)
- **SoModernGLBackend**: Canonical OpenGL implementation with per-command VBO/VAO caching
- **SoIDPickBuffer**: Offscreen FBO for GPU-based ID picking — O(1) per pick vs O(n) ray traversal
- **SoRenderManager**: Orchestrates rendering, manages draw list caching, handles GPU pick API

### FreeCAD Side (unified-renderer branch)

- **No direct OpenGL calls** — all rendering goes through Coin nodes
- **SoBrepFaceSet/EdgeSet/PointSet**: Override `render(SoModernRenderAction*)` to emit SoRenderCommands directly from cached BRep geometry
- **SoFCUnifiedSelection**: Handles preselection/selection using standard Coin pick API + direct draw list mutation for performance
- **View3DInventorViewer**: Wires up the modern renderer, foreground superimposition, interactive mode

## Enabling the Modern Renderer

Set environment variable: `FREECAD_MODERN_RENDERER=1`

Debug: `FREECAD_SHOW_ID_BUFFER=1` shows the GPU pick ID buffer

## Performance Target

**60 fps = 16.6ms render budget**

Achieved by:
- Per-command VBO/VAO caching (GL_STATIC_DRAW, only re-upload on geometry change)
- ID pass shares cached VBOs from visual pass
- Camera-aware ID buffer dirty tracking (skip re-render when stationary)
- Interactive mode skips ID buffer during orbit/pan/zoom
- Direct draw list mutation for preselection highlight (no scene traversal)
- Direct draw list mutation for click selection (no scene traversal)
- Node sensor filtering: only structural changes (not camera/selection) trigger re-traversal
- On-demand rendering (no continuous redraw when idle)

## GPU Pick System

The ID buffer renders each triangle/edge/vertex with a unique uint32 color. Pick is a pixel readback — O(1).

- **Pick LUT**: Maps sequential IDs to (commandIndex, elementType, elementIndex)
- **Type encoding**: Upper 2 bits of pick ID = element type (0=face, 1=edge, 2=vertex)
- **assemblePickedPoint()**: Constructs SoPickedPoint from LUT data for standard Coin API compatibility
- **SoHandleEventAction**: Transparently delegates to GPU pick when modern renderer is active

## Known Issues / Remaining Work

### Rendering
- (FIXED) B-spline surface corruption — chunk-based SoIRBuffer allocator
- (FIXED) Polygon offset — GL_POLYGON_OFFSET_FILL applied from render command state
- (FIXED) Standard Coin shapes — generatePrimitives fallback re-enabled
- (FIXED) Sketcher lines/circles/points — render via generatePrimitives
- (FIXED) Visibility toggle — NULL sensor triggers now invalidate draw list
- Per-vertex colors not supported (flat diffuse per command only)
- Textures not supported (SoImage, constraint icons skip modern path)
- Per-face materials not supported
- Line stipple / wireframe draw styles not supported
- SoText2 (screen-space text) skipped — needs texture/billboard support
- Delayed annotations (SoDelayedAnnotationsElement) require legacy GLRender traversal

### Selection
- SoPickedPoint created from stored command paths crashes on delete due to unrefNoDelete — hover uses resolveGpuPickIdentity instead; click path leaks SoPickedPoints as workaround
- The proper fix: redesign command path storage to use proper ref-counting
- Tree preselection/selection and progressive click-through selection: WIP stashed, needs selectAll handling

### Scene Graph Traversal
- Node sensor filters Camera and SoShape triggers for modern renderer; NULL triggers now pass through
- Scene load detected via explicit invalidateDrawList() from addViewProvider/removeViewProvider and slotFinishRestoreDocument

### Shape Coverage
- NaviCube: renders via legacy SoGLRenderAction superimposition (7ms cost)
- Grid, background gradient: rendered via legacy action (backgroundroot)
- SoImage, SoText2: skipped in modern fallback (need texture support)

## Build

### Coin
```bash
cd /Users/jso/code/FreeCAD/coin
pixi run -e default --manifest-path /Users/jso/code/FreeCAD/FreeCAD/pixi.toml cmake --build build/relwithdebinfo
pixi run -e default --manifest-path /Users/jso/code/FreeCAD/FreeCAD/pixi.toml cmake --install build/relwithdebinfo --prefix /Users/jso/code/FreeCAD/FreeCAD/.pixi/envs/default
```

### FreeCAD
```bash
cd /Users/jso/code/FreeCAD/FreeCAD
pixi run build-release
```

## Key Branches
- Coin: `unified-renderer` (based on tritao/modern-gl)
- FreeCAD: `unified-renderer`

## Tracy Profiling
Tracy is integrated for performance analysis. Key zones:
- `GLBackend::render` — total visual + ID pass
- `IDPickBuffer::renderIdPass` — ID buffer rendering
- `buildPickLUT` — indicates a scene graph re-traversal occurred
- `UnifiedSel::hover` — hover preselection time
- `cacheUpdate` — VBO/VAO cache miss handling
