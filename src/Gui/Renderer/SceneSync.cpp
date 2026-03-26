/***************************************************************************
 *   Copyright (c) 2026 FreeCAD Project Association                        *
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
 *   License along with this library; see the file COPYING.LIB. If not,   *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include "SceneSync.h"
#include "SoRenderDataCollector.h"
#include "Gui/Application.h"
#include "Gui/Selection/Selection.h"
#include "Gui/View3DInventorViewer.h"
#include "Gui/ViewProviderDocumentObject.h"

#include <Inventor/SoRenderManager.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/elements/SoCoordinateElement.h>
#include <Inventor/elements/SoModelMatrixElement.h>
#include <Inventor/elements/SoNormalElement.h>
#include <Inventor/elements/SoLazyElement.h>

#include <Base/Console.h>

#include <cmath>
#include <limits>
#include <set>

using namespace Gui;

SceneSync::SceneSync() = default;
SceneSync::~SceneSync() = default;

// -----------------------------------------------------------------------
// SoCallbackAction callback — collects geometry during traversal
// -----------------------------------------------------------------------

struct CallbackData
{
    std::vector<RenderItem> items;
};

static SoCallbackAction::Response faceSetCB(void* userData, SoCallbackAction* action, const SoNode* node)
{
    auto* data = static_cast<CallbackData*>(userData);
    auto* faceset = static_cast<const SoIndexedFaceSet*>(node);

    if (faceset->coordIndex.getNum() == 0) {
        return SoCallbackAction::CONTINUE;
    }

    SoState* state = action->getState();
    const SoCoordinateElement* coordElem = SoCoordinateElement::getInstance(state);
    if (!coordElem || coordElem->getNum() <= 0) {
        return SoCallbackAction::CONTINUE;
    }

    int numVerts = coordElem->getNum();
    const int32_t* cindices = faceset->coordIndex.getValues(0);
    int numCIndices = faceset->coordIndex.getNum();

    // Quick validation: check first few indices
    bool valid = true;
    for (int i = 0; i < std::min(numCIndices, 10); i++) {
        if (cindices[i] >= numVerts) {
            valid = false;
            break;
        }
    }
    if (!valid) {
        return SoCallbackAction::CONTINUE;
    }

    RenderItem item;
    item.shapeNode = const_cast<SoNode*>(node);
    item.type = RenderItem::Triangles;

    // Geometry from traversal state
    item.vertices = reinterpret_cast<const float*>(&coordElem->get3(0));
    item.numVertices = numVerts;
    item.coordIndices = cindices;
    item.numCoordIndices = numCIndices;

    const SoNormalElement* normElem = SoNormalElement::getInstance(state);
    if (normElem && normElem->getNum() >= numVerts) {
        item.normals = reinterpret_cast<const float*>(&normElem->get(0));
        item.numNormals = normElem->getNum();
    }

    item.modelMatrix = SoModelMatrixElement::get(state);

    // Material from traversal state
    const SoLazyElement* lazyElem = SoLazyElement::getInstance(state);
    if (lazyElem) {
        item.diffuseColor = lazyElem->getDiffuse(state, 0);
        item.transparency = lazyElem->getTransparency(state, 0);
        item.emissiveColor = lazyElem->getEmissive(state);
    }

    static int transpLog = 0;
    if (transpLog < 5 && item.transparency > 0.001f) {
        Base::Console().message(
            "SceneSync: transparent item: transp=%.4f verts=%d\n",
            item.transparency,
            numVerts
        );
        transpLog++;
    }

    data->items.push_back(std::move(item));
    return SoCallbackAction::CONTINUE;
}

// -----------------------------------------------------------------------
// SceneSync
// -----------------------------------------------------------------------

void SceneSync::sync(View3DInventorViewer* viewer, SceneRenderer* renderer)
{
    if (!viewer || !renderer) {
        return;
    }

    SoNode* sceneRoot = viewer->getSceneGraph();
    if (!sceneRoot) {
        return;
    }

    // Only run the collection pass when the scene has changed.
    // Camera-only changes reuse existing mesh entries.
    if (!needsFullSync) {
        return;
    }

    // Use SoCallbackAction for geometry collection — it doesn't touch GL
    // state and respects SoSwitch nodes for visibility.
    CallbackData cbData;
    SoCallbackAction cba;
    cba.addPreCallback(SoIndexedFaceSet::getClassTypeId(), faceSetCB, &cbData);
    cba.apply(sceneRoot);

    // Process collected items
    processItems(cbData.items, renderer);
    needsFullSync = false;
}

void SceneSync::processItems(const std::vector<RenderItem>& items, SceneRenderer* renderer)
{
    // Build set of instance keys found this frame.
    // For now, only process face geometry (Triangles). Edges and points
    // add significant overhead and are not needed for the basic view.
    // TODO: support edges/points when display mode is "Flat Lines"
    std::set<uint64_t> currentKeys;
    for (const auto& item : items) {
        if (item.type == RenderItem::Triangles) {
            currentKeys.insert(item.instanceKey());
        }
    }

    // Remove entries that are no longer visible
    for (auto it = meshEntries.begin(); it != meshEntries.end();) {
        if (currentKeys.count(it->first) == 0) {
            if (it->second.meshId != SceneRenderer::InvalidMesh) {
                renderer->remove(it->second.meshId);
            }
            it = meshEntries.erase(it);
        }
        else {
            ++it;
        }
    }

    // Add new entries and update existing ones (faces only for now)
    for (const auto& item : items) {
        if (item.type != RenderItem::Triangles) {
            continue;
        }
        uint64_t key = item.instanceKey();
        auto it = meshEntries.find(key);

        if (it != meshEntries.end()) {
            // Already submitted — update transform and material
            renderer->updateTransform(it->second.meshId, item.modelMatrix);
            renderer->updateMaterial(it->second.meshId, item.diffuseColor, item.transparency);

            // Update selection/highlight state
            bool isHighlighted = item.highlightIndex >= 0;
            bool isSelected = !item.selectedIndices.empty();

            if (isHighlighted) {
                renderer->setHighlight(it->second.meshId, item.highlightIndex, item.highlightColor);
            }
            else {
                renderer->clearHighlight(it->second.meshId);
            }

            if (isSelected) {
                std::vector<int> indices(item.selectedIndices.begin(), item.selectedIndices.end());
                renderer->setSelection(it->second.meshId, indices, item.selectionColor);
            }
            else {
                renderer->clearSelection(it->second.meshId);
            }
            continue;
        }

        // New instance — submit geometry
        MeshEntry entry;
        entry.type = item.type;

        switch (item.type) {
            case RenderItem::Triangles:
                if (item.coordIndices && item.numCoordIndices > 0) {
                    // Triangulate: coordIndices uses -1 terminated polygons
                    std::vector<int32_t> triIndices;
                    triIndices.reserve(item.numCoordIndices);
                    int polyStart = 0;
                    bool valid = true;
                    for (int i = 0; i < item.numCoordIndices; i++) {
                        if (item.coordIndices[i] < 0) {
                            int polyLen = i - polyStart;
                            for (int j = 1; j + 1 < polyLen; j++) {
                                int i0 = item.coordIndices[polyStart];
                                int i1 = item.coordIndices[polyStart + j];
                                int i2 = item.coordIndices[polyStart + j + 1];
                                if (i0 < 0 || i0 >= item.numVertices || i1 < 0
                                    || i1 >= item.numVertices || i2 < 0 || i2 >= item.numVertices) {
                                    valid = false;
                                    break;
                                }
                                triIndices.push_back(i0);
                                triIndices.push_back(i1);
                                triIndices.push_back(i2);
                            }
                            polyStart = i + 1;
                            if (!valid) {
                                break;
                            }
                        }
                    }
                    if (valid && !triIndices.empty()) {
                        // Compute normals if not provided by traversal state
                        const float* normals = item.normals;
                        std::vector<float> computedNormals;
                        if (!normals && item.vertices) {
                            computedNormals.resize(item.numVertices * 3, 0.0f);
                            for (size_t ti = 0; ti + 2 < triIndices.size(); ti += 3) {
                                int i0 = triIndices[ti];
                                int i1 = triIndices[ti + 1];
                                int i2 = triIndices[ti + 2];
                                const float* v0 = &item.vertices[i0 * 3];
                                const float* v1 = &item.vertices[i1 * 3];
                                const float* v2 = &item.vertices[i2 * 3];
                                float e1x = v1[0] - v0[0], e1y = v1[1] - v0[1], e1z = v1[2] - v0[2];
                                float e2x = v2[0] - v0[0], e2y = v2[1] - v0[1], e2z = v2[2] - v0[2];
                                float nx = e1y * e2z - e1z * e2y;
                                float ny = e1z * e2x - e1x * e2z;
                                float nz = e1x * e2y - e1y * e2x;
                                for (int vi : {i0, i1, i2}) {
                                    computedNormals[vi * 3 + 0] += nx;
                                    computedNormals[vi * 3 + 1] += ny;
                                    computedNormals[vi * 3 + 2] += nz;
                                }
                            }
                            for (int i = 0; i < item.numVertices; i++) {
                                float x = computedNormals[i * 3];
                                float y = computedNormals[i * 3 + 1];
                                float z = computedNormals[i * 3 + 2];
                                float len = std::sqrt(x * x + y * y + z * z);
                                if (len > 1e-8f) {
                                    computedNormals[i * 3] /= len;
                                    computedNormals[i * 3 + 1] /= len;
                                    computedNormals[i * 3 + 2] /= len;
                                }
                            }
                            normals = computedNormals.data();
                        }

                        entry.meshId = renderer->submitMesh(
                            item.vertices,
                            item.numVertices,
                            triIndices.data(),
                            static_cast<int>(triIndices.size()),
                            normals,
                            item.modelMatrix,
                            item.diffuseColor,
                            item.transparency
                        );
                    }
                }
                break;

            case RenderItem::Lines:
                if (item.coordIndices && item.numCoordIndices > 0) {
                    // Convert polylines to line segments
                    std::vector<int32_t> lineIndices;
                    lineIndices.reserve(item.numCoordIndices);
                    for (int i = 0; i + 1 < item.numCoordIndices; i++) {
                        if (item.coordIndices[i] >= 0 && item.coordIndices[i + 1] >= 0) {
                            lineIndices.push_back(item.coordIndices[i]);
                            lineIndices.push_back(item.coordIndices[i + 1]);
                        }
                        if (item.coordIndices[i] < 0 || item.coordIndices[i + 1] < 0) {
                            while (i + 1 < item.numCoordIndices && item.coordIndices[i + 1] < 0) {
                                i++;
                            }
                        }
                    }
                    if (!lineIndices.empty()) {
                        entry.meshId = renderer->submitLines(
                            item.vertices,
                            item.numVertices,
                            lineIndices.data(),
                            static_cast<int>(lineIndices.size()),
                            item.modelMatrix,
                            item.diffuseColor,
                            item.lineWidth
                        );
                    }
                }
                break;

            case RenderItem::Points:
                if (item.numVertices > 0) {
                    entry.meshId = renderer->submitPoints(
                        item.vertices,
                        item.numVertices,
                        item.modelMatrix,
                        item.diffuseColor,
                        item.pointSize
                    );
                }
                break;
        }

        if (entry.meshId != SceneRenderer::InvalidMesh) {
            entry.shapeNode = item.shapeNode;
            meshEntries[key] = entry;
            // Build reverse lookup for selection matching
            nodeToMeshIds[item.shapeNode].push_back(entry.meshId);
        }
    }
}

void SceneSync::invalidateAll(SceneRenderer* renderer)
{
    for (auto& [key, entry] : meshEntries) {
        if (entry.meshId != SceneRenderer::InvalidMesh) {
            renderer->remove(entry.meshId);
        }
    }
    meshEntries.clear();
    nodeToMeshIds.clear();
    needsFullSync = true;
}

void SceneSync::updateHighlighting(View3DInventorViewer* viewer, SceneRenderer* renderer)
{
    if (!viewer || !renderer || meshEntries.empty()) {
        return;
    }

    // Default highlight colors
    SbColor preselColor(0.88f, 0.88f, 0.08f);  // yellow-gold
    SbColor selColor(0.11f, 0.68f, 0.11f);     // green

    // Clear all highlights first
    for (auto& [key, entry] : meshEntries) {
        if (entry.meshId != SceneRenderer::InvalidMesh) {
            renderer->clearHighlight(entry.meshId);
            renderer->clearSelection(entry.meshId);
        }
    }

    // Get preselection — find the picked SoPath's tail node
    auto* renderManager = viewer->getSoRenderManager();
    if (!renderManager) {
        return;
    }

    // Query Selection singleton for preselection
    const auto& presel = Gui::Selection().getPreselection();
    if (presel.pObjectName && presel.pObjectName[0]) {
        // The preselection was resolved by Coin's pick action.
        // We need to find which shape node was picked. Use the
        // SoHandleEventAction's picked point list from the last event.
        // For now, highlight all mesh entries that share the preselected
        // object's shape nodes. This is approximate but fast.
        // TODO: use the actual picked SoPath for precise matching
    }

    // Get selection — highlight all meshes for selected objects
    auto selObjs = Gui::Selection().getSelectionEx();
    for (const auto& selEx : selObjs) {
        // Get the ViewProvider's shape nodes
        auto* vp = dynamic_cast<ViewProviderDocumentObject*>(
            Gui::Application::Instance->getViewProvider(selEx.getObject())
        );
        if (!vp) {
            continue;
        }
        // Search the VP's scene graph for SoIndexedFaceSet nodes
        // and highlight matching mesh entries
        SoSearchAction sa;
        sa.setType(SoIndexedFaceSet::getClassTypeId());
        sa.setInterest(SoSearchAction::ALL);
        sa.apply(vp->getRoot());
        SoPathList& paths = sa.getPaths();
        for (int i = 0; i < paths.getLength(); i++) {
            SoNode* shapeNode = paths[i]->getTail();
            auto nit = nodeToMeshIds.find(shapeNode);
            if (nit != nodeToMeshIds.end()) {
                for (auto meshId : nit->second) {
                    renderer->setSelection(meshId, {0}, selColor);
                }
            }
        }
    }
}
