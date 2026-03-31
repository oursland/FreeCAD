// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2011 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/bundles/SoMaterialBundle.h>
#include <Inventor/details/SoFaceDetail.h>
#include <Inventor/actions/SoModernRenderAction.h>
#include <Inventor/elements/SoCoordinateElement.h>
#include <Inventor/elements/SoDepthBufferElement.h>
#include <Inventor/elements/SoGLCacheContextElement.h>
#include <Inventor/elements/SoLazyElement.h>
#include <Inventor/elements/SoMaterialBindingElement.h>
#include <Inventor/elements/SoModelMatrixElement.h>
#include <Inventor/elements/SoNormalBindingElement.h>
#include <Inventor/elements/SoNormalElement.h>
#include <Inventor/elements/SoOverrideElement.h>
#include <Inventor/elements/SoShapeStyleElement.h>
#include <Inventor/elements/SoTextureEnabledElement.h>
#include <Inventor/errors/SoDebugError.h>
#include <Inventor/misc/SoState.h>

#include <App/Document.h>
#include <Base/Profiler.h>

#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/SoFCInteractiveElement.h>
#include <Gui/Selection/Selection.h>
#include <Gui/Selection/SoFCSelectionAction.h>
#include <Gui/Selection/SoFCUnifiedSelection.h>
#include <Gui/Inventor/So3DAnnotation.h>

#include "SoBrepFaceSet.h"
#include "ViewProviderExt.h"

using namespace PartGui;

SO_NODE_SOURCE(SoBrepFaceSet)

namespace
{

static void buildOverlayCoordIndex(
    std::vector<int32_t>& out,
    const int32_t* coordIndex,
    int coordIndexCount,
    const int32_t* partTriCounts,
    int partCount,
    const std::set<int>& parts,
    bool selectAll
)
{
    out.clear();
    if (!coordIndex || coordIndexCount <= 0) {
        return;
    }
    if (selectAll) {
        out.insert(out.end(), coordIndex, coordIndex + coordIndexCount);
        return;
    }
    if (!partTriCounts || partCount <= 0 || parts.empty()) {
        return;
    }

    std::vector<int32_t> face;
    face.reserve(8);

    int pos = 0;
    for (int part = 0; part < partCount && pos < coordIndexCount; ++part) {
        const bool include = (parts.find(part) != parts.end());
        const int tris = partTriCounts[part];
        for (int t = 0; t < tris && pos < coordIndexCount; ++t) {
            // Skip any stray delimiters.
            while (pos < coordIndexCount && coordIndex[pos] < 0) {
                pos++;
            }
            face.clear();
            while (pos < coordIndexCount && coordIndex[pos] >= 0) {
                face.push_back(coordIndex[pos]);
                pos++;
            }
            if (pos < coordIndexCount && coordIndex[pos] < 0) {
                // Consume one delimiter.
                pos++;
            }
            if (include && face.size() >= 3) {
                out.insert(out.end(), face.begin(), face.end());
                out.push_back(-1);
            }
        }
    }
}

static void renderOverlayFaces(
    SoGLRenderAction* action,
    SoIndexedFaceSet* faceSet,
    const std::vector<int32_t>& coordIndex,
    const SbColor& color,
    bool onTop
)
{
    if (!action || !faceSet || coordIndex.empty()) {
        return;
    }

    auto state = action->getState();
    state->push();

    SoLazyElement::setLightModel(state, SoLazyElement::BASE_COLOR);
    SoTextureEnabledElement::set(state, faceSet, false);
    SoMaterialBindingElement::set(state, SoMaterialBindingElement::OVERALL);
    SoOverrideElement::setMaterialBindingOverride(state, faceSet, true);

    if (onTop) {
        SoDepthBufferElement::set(state, FALSE, FALSE, SoDepthBufferElement::ALWAYS, SbVec2f(0.0f, 1.0f));
        SoShapeStyleElement::setTransparencyType(state, SoGLRenderAction::BLEND);
        SoLazyElement::setTransparencyType(state, SoGLRenderAction::BLEND);
    }
    else {
        SoDepthBufferElement::set(state, TRUE, FALSE, SoDepthBufferElement::LEQUAL, SbVec2f(0.0f, 1.0f));
    }

    SoLazyElement::setEmissive(state, &color);
    const uint32_t packed = color.getPackedValue(0.0f);
    SoLazyElement::setPacked(state, faceSet, 1, &packed, false);

    faceSet->coordIndex.setValues(0, static_cast<int32_t>(coordIndex.size()), coordIndex.data());
    faceSet->GLRender(action);

    state->pop();
}

}  // namespace

void SoBrepFaceSet::initClass()
{
    SO_NODE_INIT_CLASS(SoBrepFaceSet, SoIndexedFaceSet, "IndexedFaceSet");
}

SoBrepFaceSet::SoBrepFaceSet()
{
    SO_NODE_CONSTRUCTOR(SoBrepFaceSet);
    SO_NODE_ADD_FIELD(partIndex, (-1));
    SO_NODE_ADD_FIELD(highlightPartIndex, (-1));
    SO_NODE_ADD_FIELD(selectionPartIndex, (0));
    SO_NODE_ADD_FIELD(highlightColor, (SbColor(1.0f, 0.0f, 0.0f)));
    SO_NODE_ADD_FIELD(selectionColor, (SbColor(0.0f, 0.6f, 0.0f)));

    selectionPartIndex.setNum(0);

    selContext = std::make_shared<SelContext>();
    selContext2 = std::make_shared<SelContext>();
    packedColor = 0;

    overlayFaceSet = new SoIndexedFaceSet;
    overlayFaceSet->ref();
}

SoBrepFaceSet::~SoBrepFaceSet()
{
    if (overlayFaceSet) {
        overlayFaceSet->unref();
        overlayFaceSet = nullptr;
    }
}

void SoBrepFaceSet::doAction(SoAction* action)
{
    if (action->getTypeId() == Gui::SoHighlightElementAction::getClassTypeId()) {
        auto* hlaction = static_cast<Gui::SoHighlightElementAction*>(action);
        selCounter.checkAction(hlaction);
        if (!hlaction->isHighlighted()) {
            SelContextPtr ctx
                = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext, false);
            if (ctx) {
                ctx->highlightIndex = -1;
                touch();
            }
            if (viewProvider) {
                viewProvider->setFaceHighlightActive(false);
            }
            return;
        }

        const SoDetail* detail = hlaction->getElement();
        if (!detail) {
            SelContextPtr ctx = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext);
            ctx->highlightIndex = std::numeric_limits<int>::max();
            ctx->highlightColor = hlaction->getColor();
            touch();
        }
        else {
            if (!detail->isOfType(SoFaceDetail::getClassTypeId())) {
                SelContextPtr ctx
                    = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext, false);
                if (ctx) {
                    ctx->highlightIndex = -1;
                    touch();
                }
                if (viewProvider) {
                    viewProvider->setFaceHighlightActive(false);
                }
            }
            else {
                int index = static_cast<const SoFaceDetail*>(detail)->getPartIndex();
                SelContextPtr ctx = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext);
                ctx->highlightIndex = index;
                ctx->highlightColor = hlaction->getColor();
                touch();
            }
        }
        return;
    }
    else if (action->getTypeId() == Gui::SoSelectionElementAction::getClassTypeId()) {
        auto* selaction = static_cast<Gui::SoSelectionElementAction*>(action);
        switch (selaction->getType()) {
            case Gui::SoSelectionElementAction::All: {
                SelContextPtr ctx
                    = Gui::SoFCSelectionRoot::getActionContext<SelContext>(action, this, selContext);
                selCounter.checkAction(selaction, ctx);
                ctx->selectionIndex.clear();
                ctx->selectionIndex.insert(-1);
                ctx->selectionColor = selaction->getColor();
                touch();
                break;
            }
            case Gui::SoSelectionElementAction::None: {
                SelContextPtr ctx
                    = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext, false);
                if (ctx && (!ctx->selectionIndex.empty())) {
                    ctx->selectionIndex.clear();
                    touch();
                }
                break;
            }
            case Gui::SoSelectionElementAction::Append:
            case Gui::SoSelectionElementAction::Remove: {
                const SoDetail* detail = selaction->getElement();
                if (!detail || !detail->isOfType(SoFaceDetail::getClassTypeId())) {
                    if (selaction->isSecondary()) {
                        auto ctx = Gui::SoFCSelectionRoot::getActionContext<SelContext>(action, this);
                        selCounter.checkAction(selaction, ctx);
                        touch();
                    }
                    return;
                }
                int index = static_cast<const SoFaceDetail*>(detail)->getPartIndex();
                if (selaction->getType() == Gui::SoSelectionElementAction::Append) {
                    auto ctx = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext);
                    selCounter.checkAction(selaction, ctx);
                    ctx->selectionColor = selaction->getColor();
                    if (ctx->isSelectAll()) {
                        ctx->selectionIndex.clear();
                    }
                    if (ctx->selectionIndex.insert(index).second) {
                        touch();
                    }
                }
                else {
                    auto ctx
                        = Gui::SoFCSelectionRoot::getActionContext(action, this, selContext, false);
                    if (ctx && ctx->removeIndex(index)) {
                        touch();
                    }
                }
                break;
            }
            default:
                break;
        }
        return;
    }
    else if (action->getTypeId() == Gui::SoVRMLAction::getClassTypeId()) {
        // Keep materialIndex in sync when using PER_PART binding with one color per part.
        SoState* state = action->getState();
        Binding mbind = this->findMaterialBinding(state);
        if (mbind == PER_PART) {
            const SoLazyElement* mat = SoLazyElement::getInstance(state);
            const int numParts = partIndex.getNum();
            if (mat && mat->getNumDiffuse() == numParts) {
                int count = 0;
                const int32_t* indices = this->partIndex.getValues(0);
                for (int i = 0; i < numParts; i++) {
                    count += indices[i];
                }
                this->materialIndex.setNum(count);
                int32_t* matind = this->materialIndex.startEditing();
                int32_t k = 0;
                for (int i = 0; i < numParts; i++) {
                    for (int j = 0; j < indices[i]; j++) {
                        matind[k++] = i;
                    }
                }
                this->materialIndex.finishEditing();
            }
        }
    }

    inherited::doAction(action);
}

void SoBrepFaceSet::renderHighlight(SoGLRenderAction* action, SelContextPtr ctx)
{
    if (!ctx || ctx->highlightIndex < 0) {
        return;
    }

    const int32_t* partCounts = this->partIndex.getValues(0);
    const int partCount = this->partIndex.getNum();
    const int32_t* ci = this->coordIndex.getValues(0);
    const int ciCount = this->coordIndex.getNum();

    const int id = ctx->highlightIndex;
    if (id != std::numeric_limits<int>::max() && (id < 0 || id >= partCount)) {
        SoDebugError::postWarning("SoBrepFaceSet::renderHighlight", "highlightIndex out of range");
        return;
    }

    std::set<int> parts;
    const bool selectAll = (id == std::numeric_limits<int>::max());
    if (!selectAll) {
        parts.insert(id);
    }
    buildOverlayCoordIndex(overlayCoordIndex, ci, ciCount, partCounts, partCount, parts, selectAll);

    const bool onTop = Gui::Selection().isClarifySelectionActive()
        && Gui::SoDelayedAnnotationsElement::isProcessingDelayedPaths;

    renderOverlayFaces(action, overlayFaceSet, overlayCoordIndex, ctx->highlightColor, onTop);
}

void SoBrepFaceSet::renderSelection(SoGLRenderAction* action, SelContextPtr ctx, bool /*push*/)
{
    if (!ctx || ctx->selectionIndex.empty()) {
        return;
    }

    const int32_t* partCounts = this->partIndex.getValues(0);
    const int partCount = this->partIndex.getNum();
    const int32_t* ci = this->coordIndex.getValues(0);
    const int ciCount = this->coordIndex.getNum();

    if (ctx->isSelectAll()) {
        std::set<int> dummy;
        buildOverlayCoordIndex(overlayCoordIndex, ci, ciCount, partCounts, partCount, dummy, true);
        renderOverlayFaces(action, overlayFaceSet, overlayCoordIndex, ctx->selectionColor, false);
        return;
    }

    std::set<int> parts;
    for (int idx : ctx->selectionIndex) {
        if (idx >= 0 && idx < partCount) {
            parts.insert(idx);
        }
    }
    if (parts.empty()) {
        SoDebugError::postWarning("SoBrepFaceSet::renderSelection", "selectionIndex out of range");
        return;
    }

    buildOverlayCoordIndex(overlayCoordIndex, ci, ciCount, partCounts, partCount, parts, false);
    renderOverlayFaces(action, overlayFaceSet, overlayCoordIndex, ctx->selectionColor, false);
}

bool SoBrepFaceSet::overrideMaterialBinding(SoGLRenderAction* /*action*/, SelContextPtr /*ctx*/, SelContextPtr /*ctx2*/)
{
    // The legacy implementation relied on raw OpenGL state; the current rendering path uses
    // explicit overlay passes (renderSelection/renderHighlight) instead.
    return false;
}

void SoBrepFaceSet::render(SoModernRenderAction* action)
{
    if (this->coordIndex.getNum() < 3) {
        return;
    }

    SoState* state = action->getState();

    // Get coordinates from state (pushed by SoCoordinate3 node above us)
    const SoCoordinateElement* coordElem = SoCoordinateElement::getInstance(state);
    if (!coordElem) {
        return;
    }
    int numCoords = coordElem->getNum();
    if (numCoords == 0) {
        return;
    }
    const SbVec3f* coords = coordElem->getArrayPtr3();
    if (!coords) {
        return;
    }

    // Get normals from state (pushed by SoNormal node)
    const SoNormalElement* normalElem = SoNormalElement::getInstance(state);
    const SbVec3f* normals = nullptr;
    int numNormals = 0;
    if (normalElem) {
        numNormals = normalElem->getNum();
        if (numNormals > 0) {
            normals = &normalElem->get(0);
        }
    }

    // Convert coordIndex (with -1 separators) to a flat triangle index array.
    // SoBrepFaceSet only uses triangles: v0, v1, v2, -1, v3, v4, v5, -1, ...
    const int32_t* ci = this->coordIndex.getValues(0);
    int ciNum = this->coordIndex.getNum();

    // Count triangles (each group of 3 indices separated by -1)
    int numTriangles = 0;
    for (int i = 0; i + 3 <= ciNum; i += 4) {
        if (ci[i] >= 0 && ci[i + 1] >= 0 && ci[i + 2] >= 0) {
            numTriangles++;
        }
    }
    if (numTriangles == 0) {
        return;
    }

    // Allocate storage from the action's per-frame pool
    int indexCount = numTriangles * 3;
    auto* indices = static_cast<uint32_t*>(
        action->allocateGeometryStorage(indexCount * sizeof(uint32_t))
    );

    int idx = 0;
    for (int i = 0; i + 3 <= ciNum; i += 4) {
        if (ci[i] >= 0 && ci[i + 1] >= 0 && ci[i + 2] >= 0) {
            indices[idx++] = static_cast<uint32_t>(ci[i]);
            indices[idx++] = static_cast<uint32_t>(ci[i + 1]);
            indices[idx++] = static_cast<uint32_t>(ci[i + 2]);
        }
    }

    // Build the render command
    SoRenderCommand cmd = {};
    cmd.selection.highlightElement = -1;

    cmd.geometry.topology = SO_TOPOLOGY_TRIANGLES;
    cmd.geometry.vertexCount = static_cast<uint32_t>(numCoords);
    cmd.geometry.normalCount = static_cast<uint32_t>(numNormals);
    cmd.geometry.indexCount = static_cast<uint32_t>(indexCount);
    cmd.geometry.positions = reinterpret_cast<const float*>(coords);
    // Normals cover the face tessellation vertices. The coordinate node may
    // include extra vertices for edges/points (numCoords > numNormals), but
    // coordIndex only references face vertices which are within normal range.
    cmd.geometry.normals = (normals && numNormals > 0) ? reinterpret_cast<const float*>(normals)
                                                       : nullptr;
    cmd.geometry.indices = indices;
    cmd.geometry.vertexStride = sizeof(SbVec3f);
    cmd.geometry.texcoords = nullptr;
    cmd.geometry.colors = nullptr;
    cmd.geometry.texcoordStride = 0;

    // Material and render state from current traversal state
    SoModernIR::fillMaterialFromState(state, cmd.material);
    SoModernIR::fillRenderStateFromState(state, cmd.state);
    cmd.modelMatrix = SoModelMatrixElement::get(state);

    // Render pass
    bool transparent = SoModernIR::isMaterialTransparent(cmd.material);
    cmd.pass = transparent ? SO_RENDERPASS_TRANSPARENT : SO_RENDERPASS_OPAQUE;
    cmd.sortKey = SoIRComputeSortKey(cmd, static_cast<uint32_t>(cmd.pass), 0);
    cmd.userData = this;

    // Pick identity: "docName\tobjName\tsubPath" for resolvePickIdentity()
    // Uses the SoFCSelectionRoot action stack to build the full sub-object path
    // (Assembly Link → Part → Body → element).
    {
        std::string docName, objName, subPath;
        auto* guiDoc = viewProvider
            ? Gui::Application::Instance->getDocument(viewProvider->getObject()->getDocument())
            : nullptr;
        if (guiDoc
            && Gui::SoFCSelectionRoot::getSelectionPath(action, guiDoc, docName, objName, subPath)) {
            cmd.pick.pickIdentity = docName + "\t" + objName + "\t" + subPath;
        }
        else if (viewProvider && viewProvider->getObject()) {
            // Fallback for objects not inside a selection root
            auto* obj = viewProvider->getObject();
            if (obj->getDocument() && obj->getNameInDocument()) {
                cmd.pick.pickIdentity = std::string(obj->getDocument()->getName()) + "\t"
                    + obj->getNameInDocument() + "\t";
            }
        }
    }

    // Pick data: per-face ranges from partIndex for GPU picking.
    // Each partIndex entry = number of triangles for that BRep face.
    // Convert to EBO offsets for the pick LUT.
    int numParts = this->partIndex.getNum();
    if (numParts > 0) {
        const int32_t* pi = this->partIndex.getValues(0);
        cmd.pick.faceStart.resize(numParts);
        cmd.pick.faceCount.resize(numParts);
        int triOffset = 0;
        for (int i = 0; i < numParts; i++) {
            cmd.pick.faceStart[i] = triOffset * 3;
            cmd.pick.faceCount[i] = pi[i] * 3;
            triOffset += pi[i];
        }
    }

    // Read highlight and selection from the context map.
    // Fall back to selContext when getRenderContext returns null (SelStack key mismatch).
    {
        SelContextPtr ctx2;
        SelContextPtr ctx = Gui::SoFCSelectionRoot::getRenderContext(this, selContext, ctx2);
        if (!ctx) {
            ctx = selContext;
        }

        // Log whenever ctx has ANY state
        if ((ctx && (ctx->highlightIndex != -1 || !ctx->selectionIndex.empty()))) {
            ZoneScopedN("BrepFaceSet::ctxState");
            char buf[256];
            std::snprintf(
                buf,
                sizeof(buf),
                "ctx=%p hl=%d sel=%zu same=%d selCtx=%p scHl=%d scSel=%zu",
                (void*)ctx.get(),
                ctx ? ctx->highlightIndex : -99,
                ctx ? ctx->selectionIndex.size() : 0,
                (ctx.get() == selContext.get()) ? 1 : 0,
                (void*)selContext.get(),
                selContext ? selContext->highlightIndex : -99,
                selContext ? selContext->selectionIndex.size() : 0
            );
            ZoneText(buf, std::strlen(buf));
        }

        if (ctx) {
            if (ctx->isHighlighted()) {
                if (ctx->highlightIndex >= 0 && !ctx->isHighlightAll()
                    && ctx->highlightIndex < partIndex.getNum()) {
                    cmd.selection.highlightElement = ctx->highlightIndex;
                    SbColor hlc = ctx->highlightColor;
                    cmd.selection.highlightColor.setValue(hlc[0], hlc[1], hlc[2], 1.0f);
                    Base::Console().warning(
                        "ModernRender: HL face=%d/%d ctx=%p this=%p id=%s\n",
                        ctx->highlightIndex,
                        partIndex.getNum(),
                        (void*)ctx.get(),
                        (void*)this,
                        cmd.pick.pickIdentity.c_str()
                    );
                }
                else {
                    Base::Console().warning(
                        "ModernRender: HL SKIPPED hlIdx=%d hlAll=%d parts=%d id=%s\n",
                        ctx->highlightIndex,
                        ctx->isHighlightAll() ? 1 : 0,
                        partIndex.getNum(),
                        cmd.pick.pickIdentity.c_str()
                    );
                }
            }
            if (!ctx->selectionIndex.empty()) {
                if (!ctx->isSelectAll()) {
                    for (int idx : ctx->selectionIndex) {
                        if (idx >= 0) {
                            cmd.selection.selectedElements.push_back(idx);
                        }
                    }
                    SbColor slc = ctx->selectionColor;
                    cmd.selection.selectionColor.setValue(slc[0], slc[1], slc[2], 0.8f);
                    Base::Console().warning(
                        "ModernRender: SEL faces=%zu id=%s\n",
                        ctx->selectionIndex.size(),
                        cmd.pick.pickIdentity.c_str()
                    );
                }
                else {
                    Base::Console().warning(
                        "ModernRender: SEL SKIPPED (selectAll) selSz=%zu id=%s\n",
                        ctx->selectionIndex.size(),
                        cmd.pick.pickIdentity.c_str()
                    );
                }
            }
        }
    }

    action->getMutableDrawList().addCommand(cmd);
    int cmdIdx = action->getMutableDrawList().getNumCommands() - 1;

    // Verify highlight was preserved through addCommand
    {
        auto& stored = action->getMutableDrawList().getCommand(cmdIdx);
        if (stored.selection.highlightElement != -1) {
            ZoneScopedN("BUG: hl after add");
            char buf[128];
            std::snprintf(
                buf,
                sizeof(buf),
                "cmd=%d hl=%d (should be -1)",
                cmdIdx,
                stored.selection.highlightElement
            );
            ZoneText(buf, std::strlen(buf));
        }
    }

    // Store scene path for this command so GPU pick can retrieve it later
    action->storeCommandPath(cmdIdx, action->getCurPath());
}

void SoBrepFaceSet::GLRender(SoGLRenderAction* action)
{
    ZoneScoped;

    if (this->coordIndex.getNum() < 3) {
        return;
    }

    SelContextPtr ctx2;
    SelContextPtr ctx = Gui::SoFCSelectionRoot::getRenderContext(this, selContext, ctx2);
    const bool hasOverlayFields = (highlightPartIndex.getValue() >= 0)
        || (selectionPartIndex.getNum() > 0);
    if (!hasOverlayFields && ctx2 && ctx2->selectionIndex.empty()) {
        return;
    }
    if (selContext2->checkGlobal(ctx)) {
        ctx = selContext2;
    }
    if (ctx && (ctx->selectionIndex.empty() && ctx->highlightIndex < 0)) {
        ctx.reset();
    }

    auto state = action->getState();
    selCounter.checkRenderCache(state);

    const bool hasContextHighlight = ctx && ctx->isHighlighted() && !ctx->isHighlightAll()
        && ctx->highlightIndex >= 0 && ctx->highlightIndex < partIndex.getNum();

    // Clarify selection: render highlight as delayed annotation on top.
    if (Gui::Selection().isClarifySelectionActive() && hasContextHighlight) {
        if (!Gui::SoDelayedAnnotationsElement::isProcessingDelayedPaths) {
            if (viewProvider) {
                viewProvider->setFaceHighlightActive(true);
            }
            const SoPath* currentPath = action->getCurPath();
            Gui::SoDelayedAnnotationsElement::addDelayedPath(state, currentPath->copy(), 100);
            return;
        }
        inherited::GLRender(action);
        renderHighlight(action, ctx);
        return;
    }

    SoMaterialBundle mb(action);
    mb.sendFirst();
    if (!this->shouldGLRender(action)) {
        return;
    }

    inherited::GLRender(action);

    // Selection first, highlight on top.
    if (ctx2 && !ctx2->selectionIndex.empty()) {
        renderSelection(action, ctx2, false);
    }
    if (ctx && !ctx->selectionIndex.empty()) {
        renderSelection(action, ctx);
    }
    renderHighlight(action, ctx);

    // Optional overlay rendering for deterministic tests (and programmatic usage).
    const int selNum = selectionPartIndex.getNum();
    if (selNum > 0) {
        SelContextPtr octx = std::make_shared<SelContext>();
        octx->selectionColor = selectionColor.getValue();
        const int32_t* vals = selectionPartIndex.getValues(0);
        for (int i = 0; i < selNum; i++) {
            octx->selectionIndex.insert(vals[i]);
        }
        renderSelection(action, octx);
    }
    const int hl = highlightPartIndex.getValue();
    if (hl >= 0) {
        SelContextPtr octx = std::make_shared<SelContext>();
        octx->highlightIndex = hl;
        octx->highlightColor = highlightColor.getValue();
        renderHighlight(action, octx);
    }
}

void SoBrepFaceSet::GLRenderBelowPath(SoGLRenderAction* action)
{
    inherited::GLRenderBelowPath(action);
}

void SoBrepFaceSet::generatePrimitives(SoAction* action)
{
    inherited::generatePrimitives(action);
}

void SoBrepFaceSet::getBoundingBox(SoGetBoundingBoxAction* action)
{
    inherited::getBoundingBox(action);
}

SoDetail* SoBrepFaceSet::createTriangleDetail(
    SoRayPickAction* action,
    const SoPrimitiveVertex* v1,
    const SoPrimitiveVertex* v2,
    const SoPrimitiveVertex* v3,
    SoPickedPoint* pp
)
{
    SoDetail* detail = inherited::createTriangleDetail(action, v1, v2, v3, pp);
    const int32_t* indices = this->partIndex.getValues(0);
    const int num = this->partIndex.getNum();
    if (indices) {
        auto* face_detail = static_cast<SoFaceDetail*>(detail);
        const int index = face_detail->getFaceIndex();
        int count = 0;
        for (int i = 0; i < num; i++) {
            count += indices[i];
            if (index < count) {
                face_detail->setPartIndex(i);
                break;
            }
        }
    }
    return detail;
}

SoBrepFaceSet::Binding SoBrepFaceSet::findMaterialBinding(SoState* const state) const
{
    Binding binding = OVERALL;
    const auto matbind = SoMaterialBindingElement::get(state);

    switch (matbind) {
        case SoMaterialBindingElement::OVERALL:
            binding = OVERALL;
            break;
        case SoMaterialBindingElement::PER_VERTEX:
            binding = PER_VERTEX;
            break;
        case SoMaterialBindingElement::PER_VERTEX_INDEXED:
            binding = PER_VERTEX_INDEXED;
            break;
        case SoMaterialBindingElement::PER_PART:
            binding = PER_PART;
            break;
        case SoMaterialBindingElement::PER_FACE:
            binding = PER_FACE;
            break;
        case SoMaterialBindingElement::PER_PART_INDEXED:
            binding = PER_PART_INDEXED;
            break;
        case SoMaterialBindingElement::PER_FACE_INDEXED:
            binding = PER_FACE_INDEXED;
            break;
        default:
            break;
    }
    return binding;
}

SoBrepFaceSet::Binding SoBrepFaceSet::findNormalBinding(SoState* const state) const
{
    Binding binding = PER_VERTEX_INDEXED;
    const auto normbind = static_cast<SoNormalBindingElement::Binding>(
        SoNormalBindingElement::get(state)
    );

    switch (normbind) {
        case SoNormalBindingElement::OVERALL:
            binding = OVERALL;
            break;
        case SoNormalBindingElement::PER_VERTEX:
            binding = PER_VERTEX;
            break;
        case SoNormalBindingElement::PER_VERTEX_INDEXED:
            binding = PER_VERTEX_INDEXED;
            break;
        case SoNormalBindingElement::PER_PART:
            binding = PER_PART;
            break;
        case SoNormalBindingElement::PER_FACE:
            binding = PER_FACE;
            break;
        case SoNormalBindingElement::PER_PART_INDEXED:
            binding = PER_PART_INDEXED;
            break;
        case SoNormalBindingElement::PER_FACE_INDEXED:
            binding = PER_FACE_INDEXED;
            break;
        default:
            break;
    }
    return binding;
}
