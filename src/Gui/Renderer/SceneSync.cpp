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
#include "Gui/View3DInventorViewer.h"
#include "Gui/ViewProvider.h"
#include "Gui/ViewProviderGeometryObject.h"

#include "Gui/Selection/Selection.h"
#include "Gui/Selection/SoFCSelection.h"

#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/elements/SoCoordinateElement.h>
#include <Inventor/elements/SoModelMatrixElement.h>
#include <Inventor/elements/SoLazyElement.h>
#include <Inventor/elements/SoNormalElement.h>
#include <Inventor/SbMatrix.h>

#include <Base/Console.h>

#include <cmath>

using namespace Gui;

SceneSync::SceneSync() = default;
SceneSync::~SceneSync() = default;

// -----------------------------------------------------------------------
// SoCallbackAction-based scene graph walk
// -----------------------------------------------------------------------

/// Data collected for each visible SoIndexedFaceSet found during traversal.
struct FaceSetData
{
    // Key: the SoIndexedFaceSet pointer + model matrix form a unique identity
    // for this particular instance (same node, different link = different instance)
    SoIndexedFaceSet* node = nullptr;

    // Geometry from the traversal state
    std::vector<float> vertices;  // interleaved xyz
    std::vector<float> normals;   // interleaved xyz (may be computed)
    std::vector<int32_t> triIndices;
    int numVerts = 0;

    // Transform & material from traversal state
    SbMatrix modelMatrix;
    SbColor diffuseColor {0.8f, 0.8f, 0.8f};
    float transparency = 0.0f;

    // Object identity from SoFCSelection ancestor (for selection matching)
    std::string objectName;

    /// Unique key: node pointer + matrix hash to distinguish linked instances
    uint64_t instanceKey() const
    {
        auto ptr = reinterpret_cast<uintptr_t>(node);
        // Mix in a few matrix elements to distinguish same-node different-transform
        uint32_t mh = 0;
        const float* m = modelMatrix[0];
        for (int i = 0; i < 16; i++) {
            uint32_t bits;
            std::memcpy(&bits, &m[i], sizeof(bits));
            mh ^= bits + 0x9e3779b9 + (mh << 6) + (mh >> 2);
        }
        return (static_cast<uint64_t>(ptr) << 32) | mh;
    }
};

/// Callback context passed through SoCallbackAction
struct SyncCallbackData
{
    std::vector<FaceSetData> facesets;
};

/// Called for each SoIndexedFaceSet encountered during visible traversal
static SoCallbackAction::Response faceSetCB(void* userData, SoCallbackAction* action, const SoNode* node)
{
    auto* data = static_cast<SyncCallbackData*>(userData);
    auto* faceset = const_cast<SoIndexedFaceSet*>(static_cast<const SoIndexedFaceSet*>(node));

    if (faceset->coordIndex.getNum() == 0) {
        return SoCallbackAction::CONTINUE;
    }

    SoState* state = action->getState();

    // Get coordinates from traversal state
    const SoCoordinateElement* coordElem = SoCoordinateElement::getInstance(state);
    if (!coordElem) {
        return SoCallbackAction::CONTINUE;
    }
    int numVerts = coordElem->getNum();
    if (numVerts <= 0) {
        return SoCallbackAction::CONTINUE;
    }

    // Get indices and triangulate
    const int32_t* cindices = faceset->coordIndex.getValues(0);
    int numCIndices = faceset->coordIndex.getNum();

    std::vector<int32_t> triIndices;
    triIndices.reserve(numCIndices);
    int polyStart = 0;
    bool valid = true;
    for (int i = 0; i < numCIndices; i++) {
        if (cindices[i] < 0) {
            int polyLen = i - polyStart;
            for (int j = 1; j + 1 < polyLen; j++) {
                int i0 = cindices[polyStart];
                int i1 = cindices[polyStart + j];
                int i2 = cindices[polyStart + j + 1];
                if (i0 >= numVerts || i1 >= numVerts || i2 >= numVerts || i0 < 0 || i1 < 0 || i2 < 0) {
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
    if (!valid || triIndices.empty()) {
        return SoCallbackAction::CONTINUE;
    }

    FaceSetData fd;
    fd.node = faceset;
    fd.numVerts = numVerts;

    // Copy vertex positions
    fd.vertices.resize(numVerts * 3);
    for (int i = 0; i < numVerts; i++) {
        SbVec3f v = coordElem->get3(i);
        fd.vertices[i * 3 + 0] = v[0];
        fd.vertices[i * 3 + 1] = v[1];
        fd.vertices[i * 3 + 2] = v[2];
    }

    // Get normals from state, or compute them
    const SoNormalElement* normElem = SoNormalElement::getInstance(state);
    if (normElem && normElem->getNum() >= numVerts) {
        fd.normals.resize(numVerts * 3);
        for (int i = 0; i < numVerts; i++) {
            SbVec3f n = normElem->get(i);
            fd.normals[i * 3 + 0] = n[0];
            fd.normals[i * 3 + 1] = n[1];
            fd.normals[i * 3 + 2] = n[2];
        }
    }
    else {
        // Compute per-vertex normals by averaging face normals
        fd.normals.resize(numVerts * 3, 0.0f);
        for (size_t ti = 0; ti + 2 < triIndices.size(); ti += 3) {
            int i0 = triIndices[ti], i1 = triIndices[ti + 1], i2 = triIndices[ti + 2];
            float* v0 = &fd.vertices[i0 * 3];
            float* v1 = &fd.vertices[i1 * 3];
            float* v2 = &fd.vertices[i2 * 3];
            float e1x = v1[0] - v0[0], e1y = v1[1] - v0[1], e1z = v1[2] - v0[2];
            float e2x = v2[0] - v0[0], e2y = v2[1] - v0[1], e2z = v2[2] - v0[2];
            float nx = e1y * e2z - e1z * e2y;
            float ny = e1z * e2x - e1x * e2z;
            float nz = e1x * e2y - e1y * e2x;
            for (int vi : {i0, i1, i2}) {
                fd.normals[vi * 3 + 0] += nx;
                fd.normals[vi * 3 + 1] += ny;
                fd.normals[vi * 3 + 2] += nz;
            }
        }
        for (int i = 0; i < numVerts; i++) {
            float x = fd.normals[i * 3 + 0], y = fd.normals[i * 3 + 1], z = fd.normals[i * 3 + 2];
            float len = std::sqrt(x * x + y * y + z * z);
            if (len > 1e-8f) {
                fd.normals[i * 3 + 0] /= len;
                fd.normals[i * 3 + 1] /= len;
                fd.normals[i * 3 + 2] /= len;
            }
        }
    }

    fd.triIndices = std::move(triIndices);

    // Get model matrix from traversal state
    fd.modelMatrix = SoModelMatrixElement::get(state);

    // Walk up the current path to find the nearest SoFCSelection ancestor,
    // which carries the object name for selection matching.
    const SoPath* curPath = action->getCurPath();
    if (curPath) {
        for (int pi = curPath->getLength() - 1; pi >= 0; pi--) {
            SoNode* pathNode = curPath->getNodeFromTail(pi);
            if (pathNode->isOfType(Gui::SoFCSelection::getClassTypeId())) {
                auto* sel = static_cast<Gui::SoFCSelection*>(pathNode);
                fd.objectName = sel->objectName.getValue().getString();
                break;
            }
        }
    }

    // Get material from traversal state
    const SoLazyElement* lazyElem = SoLazyElement::getInstance(state);
    if (lazyElem) {
        fd.diffuseColor = lazyElem->getDiffuse(state, 0);
        fd.transparency = lazyElem->getTransparency(state, 0);
    }

    data->facesets.push_back(std::move(fd));
    return SoCallbackAction::CONTINUE;
}

// -----------------------------------------------------------------------
// SceneSync implementation
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

    // Walk the visible scene graph every frame to detect changes.
    SyncCallbackData cbData;
    SoCallbackAction cba;
    cba.addPreCallback(SoIndexedFaceSet::getClassTypeId(), faceSetCB, &cbData);
    cba.apply(sceneRoot);

    // Build set of instance keys found this frame
    std::set<uint64_t> currentKeys;
    for (auto& fd : cbData.facesets) {
        currentKeys.insert(fd.instanceKey());
    }

    // Remove entries that are no longer visible
    for (auto it = meshEntries.begin(); it != meshEntries.end();) {
        if (currentKeys.count(it->first) == 0) {
            if (it->second.faceMeshId != SceneRenderer::InvalidMesh) {
                renderer->remove(it->second.faceMeshId);
            }
            it = meshEntries.erase(it);
        }
        else {
            ++it;
        }
    }

    // Add new entries and update existing ones
    for (auto& fd : cbData.facesets) {
        uint64_t key = fd.instanceKey();
        auto it = meshEntries.find(key);

        if (it != meshEntries.end()) {
            // Already submitted — update transform and material in case they changed
            renderer->updateTransform(it->second.faceMeshId, fd.modelMatrix);
            renderer->updateMaterial(it->second.faceMeshId, fd.diffuseColor, fd.transparency);
            continue;
        }

        // New instance — submit geometry
        SyncEntry entry;
        entry.faceMeshId = renderer->submitMesh(
            fd.vertices.data(),
            fd.numVerts,
            fd.triIndices.data(),
            static_cast<int>(fd.triIndices.size()),
            fd.normals.data(),
            fd.modelMatrix,
            fd.diffuseColor,
            fd.transparency
        );
        meshEntries[key] = entry;
    }

    // Apply preselection and selection highlighting.
    // Query the Selection singleton for the current state and apply
    // override colors to matching mesh entries.
    const auto& presel = Gui::Selection().getPreselection();
    std::string preselObj = presel.pObjectName ? presel.pObjectName : "";

    // Get selected objects for the active document
    auto selObjs = Gui::Selection().getSelection();
    std::set<std::string> selectedNames;
    for (const auto& sel : selObjs) {
        if (sel.FeatName) {
            selectedNames.insert(sel.FeatName);
        }
    }

    // Default highlight colors (FreeCAD defaults)
    SbColor preselColor(0.88f, 0.88f, 0.08f);  // yellow-gold for preselection
    SbColor selColor(0.11f, 0.68f, 0.11f);     // green for selection

    // Build a map from object name to faceset keys for quick lookup
    std::map<std::string, std::vector<uint64_t>> nameToKeys;
    for (auto& fd : cbData.facesets) {
        if (!fd.objectName.empty()) {
            nameToKeys[fd.objectName].push_back(fd.instanceKey());
        }
    }

    // Clear all highlights first, then apply current state
    for (auto& [key, entry] : meshEntries) {
        if (entry.faceMeshId != SceneRenderer::InvalidMesh) {
            renderer->clearHighlight(entry.faceMeshId);
            renderer->clearSelection(entry.faceMeshId);
        }
    }

    // Apply selection highlighting
    for (const auto& name : selectedNames) {
        auto nit = nameToKeys.find(name);
        if (nit != nameToKeys.end()) {
            for (uint64_t key : nit->second) {
                auto eit = meshEntries.find(key);
                if (eit != meshEntries.end() && eit->second.faceMeshId != SceneRenderer::InvalidMesh) {
                    renderer->setSelection(eit->second.faceMeshId, {0}, selColor);
                }
            }
        }
    }

    // Apply preselection highlighting (overrides selection visually)
    if (!preselObj.empty()) {
        auto nit = nameToKeys.find(preselObj);
        if (nit != nameToKeys.end()) {
            for (uint64_t key : nit->second) {
                auto eit = meshEntries.find(key);
                if (eit != meshEntries.end() && eit->second.faceMeshId != SceneRenderer::InvalidMesh) {
                    renderer->setHighlight(eit->second.faceMeshId, 0, preselColor);
                }
            }
        }
    }
}

void SceneSync::invalidateAll()
{
    meshEntries.clear();
}
