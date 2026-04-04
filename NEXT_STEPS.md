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

## Phase 1: Feature Parity — Critical

### ~~1.1 Per-face materials for BRep shapes~~ DONE

### 1.2 Custom FreeCAD nodes missing modern renderer support
These nodes only implement `GLRender()`:
- ~~**SoFCBackgroundGradient**~~ DONE — migrated to modern renderer
- **SoFCBoundingBox** — bounding box display missing
- **SoDrawingGrid** — grid display missing
- **So3DAnnotation** — 3D annotations missing (delayed annotation path)
- **Fix pattern:** Add `doAction()` override with child traversal (same as SoDatumLabel/SoShapeScale)

### ~~1.3 NaviCube modernization~~ DONE
Known issue: NaviCube back faces visible (needs 3D overlay sub-pass with depth).

### 1.4 Dragger nodes
SoTransformDragger, SoLinearDragger, SoRotatorDragger only implement `GLRender()`.
- **Approach:** Add `doAction()` overrides similar to SoShapeScale pattern.

## Phase 2: Feature Parity — Medium Priority

### 2.1 Emissive material handling
Current hack: detects emissive-only materials by checking if diffuse ≈ (0.8,0.8,0.8). Fragile.
- **Approach:** Proper emissive handling in shader. Remove default-gray heuristic.

### 2.2 Multiple light sources
Shader uses only view-space headlight. Legacy supports light list.
- **Approach:** Capture lights from `SoLightElement`; pass up to 4 as uniforms.

### 2.3 Texture filtering
All textures use `GL_NEAREST`. World-space textures (SoDatumLabel text) should use `GL_LINEAR`.
- **Approach:** `GL_NEAREST` for billboard (flags & 0x2), `GL_LINEAR` for world-space.

### 2.4 Vertex highlight using marker bitmap
Selection highlight is `GL_POINTS` circle, not marker bitmap shape.
- **Approach:** Re-render billboard with highlight color in overlay pass.

## Phase 3: Refactoring — Code Quality

### 3.1 Split monolithic render() method (~438 lines)
Extract: `updateGeometryCache()`, `renderVisualPass()`, `renderSelectionOverlays()`, `renderIDBuffer()`

### 3.2 Convert drawCached lambda to method
163-line lambda captures 15+ variables. Move to `drawCommand()` method.

### 3.3 Extract magic numbers to named constants
Point sizes, alpha thresholds, shader coefficients, flag bits → config struct + enums.

### 3.4 Sync public/internal IR headers
Manual sync causes heap corruption on drift. Use single source with guards.

### 3.5 Simplify sensor callback logic
Accumulated special cases → explicit invalidation reason enum.

### 3.6 GL state management cleanup
Ad-hoc enable/disable → RAII guards or state-diff tracking.

### 3.7 Include generation in VBO cache key
Pool address reuse can match stale entries. Add generation to CacheKey struct.

## Phase 4: Refactoring — Architecture

### 4.1 Camera-dependent shapes optimization
`hasCameraDependentShapes` causes full rebuild on every notification. Track per-command instead.

### 4.2 Eliminate legacy superimposition rendering
Once NaviCube is modernized, remove legacy foreground path.

### 4.3 SoRenderCommand value-initialization
Add default member initializers to all POD fields. Remove all memset calls.

### Proper command path lifecycle
Stored command paths use `unrefNoDelete()` when cleared — dangling pointers crash on delete.
- **Current workarounds:** hover uses string-based identity, click leaks SoPickedPoints
- **Fix:** Redesign storage — store as strings/indices instead of SoPath pointers

### Remove FreeCAD-specific workarounds
- `onDirectHighlight` callback → Coin action framework
- `renderManager` pointer on SoFCUnifiedSelection → SoHandleEventAction
- `lastPickedLutIndex` → tracked inside Coin

## IMMEDIATE: SoModernGLBackend Refactoring

Full plan: `/Users/jso/.claude/plans/glittery-chasing-lemur.md`

### Step 1: Named constants ✓
- [x] Create constants header section in SoModernGLBackend.cpp
- [x] Replace all magic numbers in SoModernGLBackend.cpp (14 replacements)
- [x] Replace flag literals in SoModernIR.cpp, SoShape.cpp, SoMarkerSet.cpp, SoText2.cpp
- [x] Replace flag literals in SoIDPickBuffer.cpp (4 occurrences), SoRenderManager.cpp
- N/A: SoBrepFaceSet/EdgeSet/PointSet — no flag literals found (flags set via Coin-side IR)

**Constants added:**
- `SoModernIR.h` (both copies): `SO_MAT_HAS_TEXTURE`, `SO_MAT_IS_BILLBOARD`, `SO_FEAT_BASE_COLOR`, `SO_PARAM_CLEAR_WINDOW/INTERACTIVE/CLEAR_DEPTH/SKIP_ID`
- `SoModernGLBackend.cpp`: `AMBIENT/DIFFUSE/SPECULAR_COEFF`, `DEFAULT_SHININESS`, `HIGHLIGHT/SELECTION_ALPHA`, `ALPHA_DISCARD_THRESHOLD`, `MIN_HIGHLIGHT_POINT_SIZE`, `CACHE_UNUSED_FRAME_THRESHOLD`, `MAX_VERTEX_COUNT`, `DEFAULT_PICK_SIZE`
- GLSL shader string literals retain numeric values with comments referencing the constant names (GPU-side, can't use C++ constexpr)

### Step 2: Convert drawCached to method ✓
- [x] Extract lambda to `drawCommand()` method (202-line lambda → proper member function)
- [x] Pass view/proj/params as parameters (no lambda capture)
- [x] Fix line width and point size not restored after draw (added `glPointSize(1.0f)` / `glLineWidth(1.0f)` restore)

### Step 3: Extract render passes ✓ (partial — structural extraction done)
- [x] Extract `beginFrame()` / `endFrame()`
- [x] Extract `updateGeometryCache()`
- [x] Extract `renderBackgroundPass()`
- [x] Extract `renderMainScenePass()` — combined opaque + transparent + overlay
- [x] Extract `renderSelectionPass()`
- [x] Extract `renderIDBufferPass()`
- [x] `render()` reduced to 18-line orchestrator calling pass methods
- [x] Each pass owns its GL state setup and restore (renderBackgroundPass, renderMainScenePass now restore defaults at exit)
- [x] Pass ordering: selection rendered after main scene (before endFrame), overlays within main scene sort order

**Deferred to Step 6 (NaviCube overlay fix):**
- [ ] Split `renderMainScenePass` opaque/transparent/overlay into separate methods
- [ ] Split overlay into 2D (annotations) and 3D (NaviCube) sub-passes
- [ ] Fix NaviCube back-face rendering (3D overlay with depth clear)

**Rationale:** The current `renderMainScenePass` iterates sorted commands with state transitions (opaque→transparent→overlay). Splitting requires either multiple sort iterations or pre-partitioning commands by pass type. This restructuring is only needed for the NaviCube depth fix (Step 6), not for shader migration (Steps 4-5). Deferring avoids speculative refactoring.

### Step 4: Shader migration to GLSL 4.10
**4a: Unified shader program ✓**
- [x] Merge texture shader into main shader with `u_renderMode` (0=lit, 1=flat, 2=billboard, 3=textured)
- [x] Remove separate `texShaderProgram`, `texVAO`, all `texU*` uniform locations
- [x] Merge texcoord attribute into main VAO (setupVisualVAO)
- [x] Replace `u_emissive` boolean with `u_renderMode` float throughout
- [x] Explicit `glBindAttribLocation` before linking — macOS GLSL 1.20 reassigns attribute locations when adding new attributes, causing silent VAO mismatch

**4b: GLSL 4.10 syntax upgrade ✓ (coupled with Step 5)**
- [x] Upgrade `#version 120` → `#version 410 core` (main shader + ID pick shader)
- [x] `attribute` → `layout(location=N) in`, `varying` → `in/out`, `gl_FragColor` → `out vec4 fragColor`, `texture2D` → `texture`
- [x] Runtime Core Profile: `FREECAD_MODERN_RENDERER=1` → `QSurfaceFormat::CoreProfile` + GL 4.1 (Application.cpp, QuarterWidget.cpp)
- [x] macOS VAO fix: replaced `glGenVertexArraysAPPLE` macros with `extern "C"` standard function declarations (APPLE suffix doesn't exist in Core Profile)
- [x] GL error drain in `beginFrame()` — clears errors from legacy Coin code making deprecated calls in Core Profile

**4c: PBR material uniforms ✓**
- [x] Add `metalness`/`roughness` fields to SoMaterialData (both IR header copies)
- [x] Add `u_metalness`, `u_roughness` uniforms (default: 0.0, 0.5)
- [x] Implement GGX NDF + Fresnel-Schlick in fragment shader (mode 0)
- [x] Defaults produce Blinn-Phong-equivalent output (dielectric, moderate roughness)
- [x] Per-command PBR upload in drawCommand, defaults in fillMaterialFromState

### Step 5: Replace deprecated GL features ✓
- [x] Replace GL_POINT_SMOOTH with `gl_PointCoord` circle discard (`u_renderMode=1.5`)
- [x] Replace GL_LUMINANCE/GL_LUMINANCE_ALPHA with CPU RGBA expansion in `uploadGeometry`
- [x] Replace GL_LINE_STIPPLE with shader-based dashing — per-vertex cumulative object-space distance (`a_lineDistance` attribute), MVP-projected period conversion, 50% duty cycle
- [x] Remove GL_POINT_SMOOTH_HINT calls
- [x] Test on macOS with GL 4.1 Core context (implemented in Step 4b)

### Step 6: Pass refinement, NaviCube overlay fix, background fixes
- [ ] Split `renderMainScenePass` into `renderOpaquePass()`, `renderTransparentPass()`, `renderOverlayPass()`
- [ ] Split overlay into 2D (annotations) and 3D (NaviCube) sub-passes
- [ ] 3D overlay: clear depth, enable depth test+write, enable cull
- [ ] 2D overlay: depth disabled, no cull
- [ ] Verify NaviCube self-occlusion (back faces hidden)
- [ ] Verify annotations render on top of everything
- [ ] Verify pass ordering: selection before overlay
- [ ] Fix single-color background mode (not rendering — only gradient works)
- [ ] Fix background color intensity ("white" background appears ~50% gray)

### Step 7: Performance regression from NaviCube modernization
The NaviCube migration to the modern renderer caused a significant performance regression. Investigate and fix:
- [ ] Profile with Tracy to identify the bottleneck (draw call count, texture uploads, re-traversal)
- [ ] NaviCube commands may trigger full draw list rebuild each frame (camera-dependent shapes)
- [ ] NaviCube textures may be re-uploaded every frame instead of cached
- [ ] Consider separating NaviCube into its own draw list or render pass to avoid invalidating the main scene
- [ ] Target: NaviCube overhead < 1ms per frame (was ~7ms with legacy superimposition)

## Reference: Resume Prompt

When starting a new session:

> I'm working on the FreeCAD unified renderer (Coin3D modern OpenGL backend). Read `CLAUDE.md` in both `/Users/jso/code/FreeCAD/FreeCAD` and `/Users/jso/code/FreeCAD/coin` for full context. Check `git log --oneline -20` in both repos for recent progress. Read the refactoring plan at `/Users/jso/.claude/plans/glittery-chasing-lemur.md`. Also read `NEXT_STEPS.md` for the task checklist. Start with uncommitted changes: `git diff --stat` in both repos. Begin implementation from the task checklist.
