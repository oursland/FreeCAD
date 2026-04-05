# Next Steps for Unified Renderer

## Completed

### Core Infrastructure
- Unified shader with `u_renderMode` (0=lit, 1=flat, 1.5=point, 2=billboard, 3=textured)
- GLSL 4.10 Core Profile with `layout(location=N)` attributes
- Runtime Core Profile context: `FREECAD_MODERN_RENDERER=1` triggers GL 4.1 Core
- macOS VAO: `extern "C"` declarations replace APPLE suffix macros
- PBR lighting: GGX NDF + Fresnel-Schlick with `u_metalness`/`u_roughness` uniforms
- Render pass architecture: beginFrame → updateGeometryCache → background → opaque → transparent → selection → overlay → endFrame → ID buffer
- Overlay pass split: 2D annotations (depth disabled) + 3D overlays (NaviCube, depth clear + self-occlusion)

### Deprecated GL Removal
- `GL_POINT_SMOOTH` → `gl_PointCoord` circle discard (`u_renderMode=1.5`)
- `GL_LINE_STIPPLE` → shader-based dashing (`a_lineDistance` + MVP-projected period)
- `GL_LUMINANCE`/`GL_LUMINANCE_ALPHA` → CPU RGBA expansion
- `glLineWidth` clamped to 1.0 on Core Profile → geometry shader for screen-space quad lines
- `devicePixelRatio` plumbed from QuarterWidget through render params for proper scaling

### Performance
- Generation-based draw list caching (`sceneGeneration` + `foregroundGeneration`)
- Foreground-only re-traversal on camera changes with geometry pool save/rewind
- Sensor callback classifies camera/interactive/pendingCameraChange events
- `notifyCameraChange()` API for zoom scroll
- ID pick buffer: skip overlay commands

### Feature Parity
- Per-face materials for BRep shapes (per-part diffuse colors via SoBrepFaceSet)
- SoText2 screen-space text, SoMarkerSet bitmap markers, SoDatumLabel constraints
- SoShapeScale camera-dependent scaling, SoAutoZoomTranslation (origin axes)
- NaviCube modernized (projection-based corner positioning, per-command view/proj)
- Background gradient (SoFCBackgroundGradient with per-vertex color capture)
- Single-color background (glClear + modernBackgroundRoot management)
- Selection bounding box (wireframe from vertex positions in renderSelectionPass)
- Tree-view selection via `setDrawListSelectionByIdentity()` (pickIdentity prefix matching)
- So3DAnnotation: doAction override + rayPick bypass for stale bbox culling
- SoDrawingGrid: render override (unused node, only in Coin snapshot tests)
- Sketcher grid: color via orderedRGBA, stipple period from pattern bits, transparency adjusted
- Dragger interaction: cache init in doAction, immediate grab on press, grabber check in SoFCUnifiedSelection, ray pick fallback for mouse press, depth override for render-on-top, real-time drag updates

## Remaining Tasks

### Critical

- [ ] **Dragger camera update** — dragger position doesn't update during camera orbit/zoom while transform tool is active. Updates only after camera motion ends.
- [ ] **SoFrameLabel modern renderer support** — U/V/W dragger text labels have no modern renderer path. `SoFrameLabel` only implements `GLRender()`. Needs porting similar to SoText2/SoImage (texture-based billboard rendering).
- [ ] **SoMaterial state not captured** — `fillMaterialFromState` reads `(0,0,0)` for nodes using `SoMaterial` without `SoVertexProperty`. Workaround: set color via `orderedRGBA`. Root cause: `SoLazyElement` diffuse not propagating correctly during modern render traversal. Affects dragger colors, grid, and potentially other nodes.

### Rendering Quality

- [ ] **NaviCube transparency** — NaviCube should have alpha so 3D axes behind remain visible through cube faces.
- [ ] **LCS / datum render order** — LCS and datum items should render on top of geometry, not be occluded. Investigate annotation routing for datum view providers.
- [ ] **Line alpha tuning** — geometry shader lines slightly lighter than legacy. May need blending adjustment.
- [ ] **Body highlight during transform** — selected body should become semi-transparent/highlighted when transform tool is active (matching legacy behavior).
- [ ] **Emissive material handling** — current hack detects emissive-only by checking diffuse near (0.8,0.8,0.8). Replace with proper emissive path in PBR shader.
- [ ] **Texture filtering** — all textures use `GL_NEAREST`. World-space textures (SoDatumLabel text) should use `GL_LINEAR`.

### Medium Priority

- [ ] **Multiple light sources** — shader uses only headlight. Capture lights from SoLightElement; pass up to 4 as uniforms.
- [ ] **Vertex highlight using marker bitmap** — selection highlight is GL_POINTS circle, not marker bitmap shape.

### Architecture

- [x] **Legacy foreground path** — already eliminated for main render loop. NaviCube renders via `modernForegroundRoot` → `traverseAdditionalRoot`. The `superimpositions` block in `renderModern` only fires for Fem CreateLabels (rare). Offscreen path (`savePicture`) still uses legacy `gl.apply(foregroundroot)` — acceptable for now.
- [x] **SoRenderCommand value-initialization** — default member initializers added to all POD fields in SoGeometryDesc, SoMaterialData, SoDepthState, SoBlendState, SoRasterState, SoRenderState, SoRenderCommand. Meaningful defaults (linePattern=0xFFFF, lineWidth=1.0, diffuse=0.8 gray, etc.).
- [x] **Proper command path lifecycle** — resolved: `storeCommandPath`/`getCommandPath` are stubbed out (no-ops). Unsafe `SoPath` storage removed. Hover uses string-based `resolveGpuPickIdentity`, click uses `pickIdentity` prefix matching. Remaining minor: `gpuPickedPointList` in `SoHandleEventAction` leaks via `truncate` instead of `delete` to avoid dangling path crashes.
- [ ] **Sync public/internal IR headers** — manual sync causes heap corruption on drift. Use single source with guards.

### Remaining Legacy GL Paths
These paths still use `SoGLRenderAction` or direct OpenGL calls, bypassing the modern renderer:
- [ ] **Superimposition rendering** (`SoRenderManager::renderModern` line 703-720) — runs `glaction` on each registered superimposition after modern backend renders. Only used by Fem `CreateLabels.py`. Could be eliminated by porting Fem labels to modern renderer or by traversing superimpositions via `SoModernRenderAction`.
- [ ] **Offscreen rendering** (`View3DInventorViewer` line 2880-2900) — `savePicture`/thumbnail path creates a `SoBoxSelectionRenderAction` and applies it to backgroundroot, scene graph, and foregroundroot directly. Needs modern renderer equivalent for offscreen capture.
- [ ] **SoFrameLabel** (`SoTextLabel.cpp`) — dragger U/V/W labels use `SoFrameLabel::GLRender()` with direct GL calls for text/background rendering. No `render(SoModernRenderAction*)` override. Needs porting similar to SoText2.
- [ ] **SoFCBoundingBox ViewProvider path** (`ViewProviderGeometryObject::showBoundingBox`) — the `SoFCBoundingBox` node has a `render()` override but the ViewProvider `BoundingBox` property toggle is never used in practice. Selection bounding box is handled by `renderSelectionPass` instead. The ViewProvider path can be removed or left dormant.
- [ ] **SoVertexLayout::GLRender errors** — Coin's `SoVertexLayout` nodes in the scene graph attempt `GLRender` in Core Profile and log errors. Harmless but noisy. Consider suppressing or adding `doAction` overrides to skip GL calls for modern renderer.
- [ ] **drawAxisCross** (`View3DInventorViewer::drawAxisCross` line 2902) — legacy GL axis cross drawing after offscreen render. Not used in main render loop (modern renderer handles axes via SoAutoZoomTranslation).

### Performance

- [ ] **NaviCube command count** — reduce 100+ overlay commands to fewer batched draws. Directly impacts render(self) time (10.6ms→30.9ms) and ID pass (10ms→24ms).
- [ ] **Zoom-over-geometry invalidation** — selection events during zoom produce NULL trigger → scene invalidation (~17 per session).

## Reference: Resume Prompt

When starting a new session:

> I'm working on the FreeCAD unified renderer (Coin3D modern OpenGL backend). Read `CLAUDE.md` in both `/Users/jso/code/FreeCAD/FreeCAD` and `/Users/jso/code/FreeCAD/coin` for full context. Check `git log --oneline -20` in both repos for recent progress. Also read `NEXT_STEPS.md` for the task checklist. Start with uncommitted changes: `git diff --stat` in both repos. Begin implementation from the task checklist.
