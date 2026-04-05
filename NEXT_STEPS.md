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
- [ ] **SoRenderCommand value-initialization** — add default member initializers to all POD fields. Remove all memset calls.
- [ ] **Proper command path lifecycle** — stored paths use unrefNoDelete() → dangling pointers. Redesign as strings/indices.
- [ ] **Sync public/internal IR headers** — manual sync causes heap corruption on drift. Use single source with guards.

### Performance

- [ ] **NaviCube command count** — reduce 100+ overlay commands to fewer batched draws. Directly impacts render(self) time (10.6ms→30.9ms) and ID pass (10ms→24ms).
- [ ] **Zoom-over-geometry invalidation** — selection events during zoom produce NULL trigger → scene invalidation (~17 per session).

## Reference: Resume Prompt

When starting a new session:

> I'm working on the FreeCAD unified renderer (Coin3D modern OpenGL backend). Read `CLAUDE.md` in both `/Users/jso/code/FreeCAD/FreeCAD` and `/Users/jso/code/FreeCAD/coin` for full context. Check `git log --oneline -20` in both repos for recent progress. Also read `NEXT_STEPS.md` for the task checklist. Start with uncommitted changes: `git diff --stat` in both repos. Begin implementation from the task checklist.
