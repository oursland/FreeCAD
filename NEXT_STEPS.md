# Next Steps for Unified Renderer

## Completed

- Per-face materials (partial — per-vertex colors work, per-face via BRep partitioning pending)
- Textures: SoImage billboard, SoTexture2 world-space, dual shader (billboard vs MVP)
- Polygon offset (GL_POLYGON_OFFSET_FILL from render command state)
- Line stipple / wireframe draw styles
- Sketcher geometry rendering (lines, circles, B-splines, construction geometry)
- Per-vertex colors
- SoText2 screen-space text (glyph rasterization to textured billboard)
- SoMarkerSet bitmap markers (1-bit bitmap unpacking to RGBA billboard)
- SoDatumLabel constraint visuals (dimension lines, arrowheads, text labels)
- SoShapeScale camera-dependent scaling (datum points, rotation center)
- SoAnnotation overlay pass (depth test disabled, renders on top)
- SoAutoZoomTranslation (origin axes)
- Sketcher "infinite" axis lines (via camera-dependent re-traversal)
- Draw list generation counter (VBO cache invalidation on pool reuse)
- LINE_STRIP with discontinuity detection (consistent line stipple)
- SoGLLinePatternElement::push() NULL state crash fix
- memset UB fix on SoRenderCommand (std::vector corruption)
- Emissive color in Blinn-Phong shader (u_emissiveColor uniform)
- BASE_COLOR light model flag (flat/unlit rendering)
- Rotation center sphere (emissive color, constant size, translucent)
- Joint markers rendering on top (overlay pass)
- Sensor callback: NULL triggers pass through interactive mode
- Per-face materials for BRep shapes (per-part diffuse colors via SoBrepFaceSet)
- SoAnnotation overlay pass with depth test disabled
- Tracy profiling enabled in both Coin and FreeCAD
- Background gradient migrated to modern renderer (SoFCBackgroundGradient::doAction)
- Per-vertex color capture fix (SoVertexProperty state pop timing)
- Background render pass with identity view/proj and depth clear
- NaviCube migrated to modern renderer (projection-based corner positioning)
- Per-command view/proj matrices (enables multiple cameras in draw list)
- SoViewingMatrixElement + SoProjectionMatrixElement enabled for modern action
- Auto-clipping for modern renderer (setClippingPlanes before traversal)
- Backface culling support (SoShapeHintsElement → cullMode)
- Texture modulation (u_texModColor for NaviCube labels)
- Foreground root API (setModernForegroundRoot)
- Viewport decoupled from SoGLRenderAction (modernViewport member)
- featureFlags reset bug fixed in fillMaterialFromState
- Named constants: SO_MAT_HAS_TEXTURE/IS_BILLBOARD, SO_FEAT_BASE_COLOR, SO_PARAM_* flags, lighting coefficients
- drawCached lambda → drawCommand() method with state restore fixes
- Render pass extraction: beginFrame/endFrame, updateGeometryCache, renderBackgroundPass, renderOpaquePass, renderTransparentPass, renderSelectionPass, renderOverlayPass, renderIDBufferPass
- Unified shader: u_renderMode (0=lit, 1=flat, 1.5=point, 2=billboard, 3=textured) replaces separate texture shader
- GLSL 4.10 Core Profile: layout(location=N) in, in/out varyings, out vec4 fragColor, texture()
- Runtime Core Profile context: FREECAD_MODERN_RENDERER=1 → GL 4.1 Core (Application.cpp, QuarterWidget.cpp)
- macOS VAO: extern "C" standard function declarations replace APPLE suffix macros
- PBR lighting: GGX NDF + Fresnel-Schlick, u_metalness/u_roughness uniforms
- GL_POINT_SMOOTH → gl_PointCoord circle discard (u_renderMode=1.5)
- GL_LUMINANCE/GL_LUMINANCE_ALPHA → CPU RGBA expansion
- GL_LINE_STIPPLE → shader-based dashing (a_lineDistance attribute, MVP-projected period)
- GL_POINT_SMOOTH_HINT removed
- Overlay pass split: 2D (annotations, depth disabled) + 3D (NaviCube, depth clear + depth test)
- Pass ordering: opaque → transparent → selection → overlay
- Single-color background: glClear(GL_COLOR_BUFFER_BIT) + modernBackgroundRoot unregister on NoGradient
- Background per-vertex color capture: capturedPerVertexColor flag in SoModernPrimitiveAssembler
- Generation-based draw list caching (sceneGeneration + foregroundGeneration)
- Foreground-only re-traversal on camera changes with geometry pool save/rewind
- Sensor callback: camera/interactive/pendingCameraChange → fg-only, SoCamera type check, shape touch skip
- notifyCameraChange() API for zoom scroll
- ID pick buffer: skip overlay commands (not pickable)

## Remaining Tasks

### Feature Parity — Critical

- [x] **SoFCBoundingBox / Selection BBox** — Wireframe bounding box rendered in `renderSelectionPass` from command vertex positions. Tree-view selection implemented via `setDrawListSelectionByIdentity()` which matches commands by `pickIdentity` prefix (`docName\tobjName\t`). Whole-body selection uses element -2.
- [x] **SoDrawingGrid** — `render(SoModernRenderAction*)` override working. Grid renders on new sketches. Lines faint vs legacy (color/alpha). Pre-existing bug: grid doesn't show on existing sketches.
- [x] **So3DAnnotation** — `doAction()` override added. Traverses children directly for modern renderer.
- [ ] **Dragger nodes** — `SoDragger::doAction()` added in Coin. Draggers render + highlight on click, but translation/rotation doesn't work. Text colors incorrect. May be partially upstream dragger regression.

### Feature Parity — Medium Priority

- [ ] **Emissive material handling** — current hack detects emissive-only by checking diffuse ≈ (0.8,0.8,0.8). Replace with proper emissive handling in PBR shader.
- [ ] **Multiple light sources** — shader uses only headlight. Capture lights from SoLightElement; pass up to 4 as uniforms.
- [ ] **Texture filtering** — all textures use GL_NEAREST. World-space textures should use GL_LINEAR.
- [ ] **Vertex highlight using marker bitmap** — selection highlight is GL_POINTS circle, not marker bitmap shape.

### Code Quality

- [ ] **Sync public/internal IR headers** — manual sync causes heap corruption on drift. Use single source with guards.
- [ ] **GL state management cleanup** — ad-hoc enable/disable → RAII guards or state-diff tracking.

### Architecture

- [ ] **Eliminate legacy superimposition rendering** — NaviCube is modernized, remove legacy foreground path.
- [ ] **SoRenderCommand value-initialization** — add default member initializers to all POD fields. Remove all memset calls.
- [ ] **Proper command path lifecycle** — stored paths use unrefNoDelete() → dangling pointers. Redesign as strings/indices.
- [ ] **Remove FreeCAD-specific workarounds** — onDirectHighlight callback, renderManager pointer on SoFCUnifiedSelection, lastPickedLutIndex.

### Performance

- [ ] **render(self) regression** — 10.6ms→30.9ms from ~100 NaviCube overlay draw calls. Batch into fewer draws.
- [ ] **ID pass regression** — 10ms→24ms. Investigate remaining overhead after overlay skip.
- [ ] **Zoom-over-geometry invalidation** — selection events during zoom produce NULL trigger → scene invalidation (~17 per session).
- [ ] **NaviCube command count** — reduce 100+ commands to fewer batched draws.

## Reference: Resume Prompt

When starting a new session:

> I'm working on the FreeCAD unified renderer (Coin3D modern OpenGL backend). Read `CLAUDE.md` in both `/Users/jso/code/FreeCAD/FreeCAD` and `/Users/jso/code/FreeCAD/coin` for full context. Check `git log --oneline -20` in both repos for recent progress. Read the refactoring plan at `/Users/jso/.claude/plans/glittery-chasing-lemur.md`. Also read `NEXT_STEPS.md` for the task checklist. Start with uncommitted changes: `git diff --stat` in both repos. Begin implementation from the task checklist.
