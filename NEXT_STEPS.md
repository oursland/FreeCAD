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

### Step 1: Named constants
- [ ] Create constants header section in SoModernGLBackend.cpp
- [ ] Replace all magic numbers in SoModernGLBackend.cpp
- [ ] Replace flag literals in SoModernIR.cpp, SoShape.cpp, SoMarkerSet.cpp, SoText2.cpp
- [ ] Replace flag literals in SoBrepFaceSet.cpp, SoBrepEdgeSet.cpp, SoBrepPointSet.cpp

### Step 2: Convert drawCached to method
- [ ] Extract lambda to `drawCommand()` method
- [ ] Pass view/proj/params as parameters (no lambda capture)
- [ ] Fix line width and point size not restored after draw

### Step 3: Extract render passes
- [ ] Extract `beginFrame()` / `endFrame()`
- [ ] Extract `updateGeometryCache()`
- [ ] Extract `renderBackgroundPass()`
- [ ] Extract `renderOpaquePass()`
- [ ] Extract `renderTransparentPass()`
- [ ] Extract `renderSelectionPass()`
- [ ] Extract `renderOverlayPass()` (split 2D and 3D sub-passes)
- [ ] Extract ID buffer to `renderIDBufferPass()`
- [ ] Fix NaviCube back-face rendering (3D overlay with depth clear)
- [ ] Verify pass ordering: selection before overlay

### Step 4: Shader migration to GLSL 4.10
- [ ] Write unified PBR vertex shader
- [ ] Write unified PBR fragment shader
- [ ] Implement `u_renderMode` (flat, lit, billboard, textured)
- [ ] Add PBR material fields to SoMaterialData (metalness, roughness, AO)
- [ ] Remove separate texture shader program (merge into main)
- [ ] Implement GGX NDF + Fresnel-Schlick in fragment shader
- [ ] Verify Blinn-Phong-equivalent output with default PBR values

### Step 5: Replace deprecated GL features
- [ ] Replace GL_LINE_STIPPLE with shader-based dashing
- [ ] Replace GL_POINT_SMOOTH with gl_PointCoord circle discard
- [ ] Replace GL_LUMINANCE with GL_RED + swizzle or shader
- [ ] Test on macOS with GL 4.1 Core context

### Step 6: NaviCube overlay fix
- [ ] Split overlay into 2D (annotations) and 3D (NaviCube) sub-passes
- [ ] 3D overlay: clear depth, enable depth test+write, enable cull
- [ ] Verify NaviCube self-occlusion and annotations on top

## Reference: Resume Prompt

When starting a new session:

> I'm working on the FreeCAD unified renderer (Coin3D modern OpenGL backend). Read `CLAUDE.md` in both `/Users/jso/code/FreeCAD/FreeCAD` and `/Users/jso/code/FreeCAD/coin` for full context. Check `git log --oneline -20` in both repos for recent progress. Read the refactoring plan at `/Users/jso/.claude/plans/glittery-chasing-lemur.md`. Also read `NEXT_STEPS.md` for the task checklist. Start with uncommitted changes: `git diff --stat` in both repos. Begin implementation from the task checklist.
