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

#include <Inventor/actions/SoSearchAction.h>
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
#include <Inventor/SbMatrix.h>

#include <Base/Console.h>

using namespace Gui;

SceneSync::SceneSync() = default;
SceneSync::~SceneSync() = default;

// Find the first node of a given type under a root
static SoNode* findNodeOfType(SoNode* root, SoType type)
{
    SoSearchAction sa;
    sa.setType(type);
    sa.setInterest(SoSearchAction::FIRST);
    sa.apply(root);
    SoPath* path = sa.getPath();
    if (path) {
        return path->getTail();
    }
    return nullptr;
}

void SceneSync::sync(View3DInventorViewer* viewer, SceneRenderer* renderer)
{
    if (!viewer || !renderer) {
        return;
    }

    // Collect all visible geometry ViewProviders
    auto vpList = viewer->getViewProvidersOfType(ViewProviderGeometryObject::getClassTypeId());

    std::map<ViewProvider*, bool> seen;

    for (auto* vp : vpList) {
        seen[vp] = true;

        if (!vp->isVisible()) {
            auto it = entries.find(vp);
            if (it != entries.end()) {
                auto& entry = it->second;
                if (entry.faceMeshId != SceneRenderer::InvalidMesh) {
                    renderer->setVisible(entry.faceMeshId, false);
                }
                if (entry.lineMeshId != SceneRenderer::InvalidMesh) {
                    renderer->setVisible(entry.lineMeshId, false);
                }
                if (entry.pointMeshId != SceneRenderer::InvalidMesh) {
                    renderer->setVisible(entry.pointMeshId, false);
                }
            }
            continue;
        }

        uint32_t nodeId = vp->getRoot()->getNodeId();
        auto it = entries.find(vp);
        if (it != entries.end() && it->second.lastNodeId == nodeId) {
            if (it->second.faceMeshId != SceneRenderer::InvalidMesh) {
                renderer->setVisible(it->second.faceMeshId, true);
            }
            if (it->second.lineMeshId != SceneRenderer::InvalidMesh) {
                renderer->setVisible(it->second.lineMeshId, true);
            }
            if (it->second.pointMeshId != SceneRenderer::InvalidMesh) {
                renderer->setVisible(it->second.pointMeshId, true);
            }
            continue;
        }

        syncViewProvider(vp, renderer);
        entries[vp].lastNodeId = nodeId;
    }

    // Remove entries for VPs that no longer exist
    for (auto it = entries.begin(); it != entries.end();) {
        if (seen.find(it->first) == seen.end()) {
            auto& entry = it->second;
            if (entry.faceMeshId != SceneRenderer::InvalidMesh) {
                renderer->remove(entry.faceMeshId);
            }
            if (entry.lineMeshId != SceneRenderer::InvalidMesh) {
                renderer->remove(entry.lineMeshId);
            }
            if (entry.pointMeshId != SceneRenderer::InvalidMesh) {
                renderer->remove(entry.pointMeshId);
            }
            it = entries.erase(it);
        }
        else {
            ++it;
        }
    }
}

void SceneSync::invalidateAll()
{
    for (auto& kv : entries) {
        kv.second.lastNodeId = 0;
    }
}

void SceneSync::syncViewProvider(ViewProvider* vp, SceneRenderer* renderer)
{
    // Remove old entries
    auto& entry = entries[vp];
    if (entry.faceMeshId != SceneRenderer::InvalidMesh) {
        renderer->remove(entry.faceMeshId);
        entry.faceMeshId = SceneRenderer::InvalidMesh;
    }
    if (entry.lineMeshId != SceneRenderer::InvalidMesh) {
        renderer->remove(entry.lineMeshId);
        entry.lineMeshId = SceneRenderer::InvalidMesh;
    }
    if (entry.pointMeshId != SceneRenderer::InvalidMesh) {
        renderer->remove(entry.pointMeshId);
        entry.pointMeshId = SceneRenderer::InvalidMesh;
    }

    SoNode* root = vp->getRoot();
    if (!root) {
        return;
    }

    // Find coordinate node
    auto* coords = dynamic_cast<SoCoordinate3*>(findNodeOfType(root, SoCoordinate3::getClassTypeId()));
    if (!coords || coords->point.getNum() == 0) {
        return;
    }

    const SbVec3f* vertexData = coords->point.getValues(0);
    int numVerts = coords->point.getNum();

    // Get transform
    SbMatrix modelMatrix;
    auto* transform = dynamic_cast<SoTransform*>(findNodeOfType(root, SoTransform::getClassTypeId()));
    if (transform) {
        SbVec3f t = transform->translation.getValue();
        SbRotation r = transform->rotation.getValue();
        SbVec3f s = transform->scaleFactor.getValue();
        SbRotation so = transform->scaleOrientation.getValue();
        SbVec3f c = transform->center.getValue();
        modelMatrix.setTransform(t, r, s, so, c);
    }
    else {
        modelMatrix.makeIdentity();
    }

    // Get material color
    SbColor diffuseColor(0.8f, 0.8f, 0.8f);
    float transparency = 0.0f;
    auto* material = dynamic_cast<SoMaterial*>(findNodeOfType(root, SoMaterial::getClassTypeId()));
    if (material) {
        if (material->diffuseColor.getNum() > 0) {
            diffuseColor = material->diffuseColor[0];
        }
        if (material->transparency.getNum() > 0) {
            transparency = material->transparency[0];
        }
    }

    // Submit face geometry (SoIndexedFaceSet or subclass like SoBrepFaceSet)
    auto* faceset = dynamic_cast<SoIndexedFaceSet*>(
        findNodeOfType(root, SoIndexedFaceSet::getClassTypeId())
    );
    if (faceset && faceset->coordIndex.getNum() > 0) {
        const int32_t* cindices = faceset->coordIndex.getValues(0);
        int numCIndices = faceset->coordIndex.getNum();

        std::vector<int32_t> triIndices;
        triIndices.reserve(numCIndices);
        for (int i = 0; i + 2 < numCIndices;) {
            if (cindices[i] >= 0 && cindices[i + 1] >= 0 && cindices[i + 2] >= 0) {
                triIndices.push_back(cindices[i]);
                triIndices.push_back(cindices[i + 1]);
                triIndices.push_back(cindices[i + 2]);
            }
            while (i < numCIndices && cindices[i] >= 0) {
                i++;
            }
            i++;
        }

        auto* norm = dynamic_cast<SoNormal*>(findNodeOfType(root, SoNormal::getClassTypeId()));
        const float* normalData = nullptr;
        if (norm && norm->vector.getNum() == numVerts) {
            normalData = reinterpret_cast<const float*>(norm->vector.getValues(0));
        }

        if (!triIndices.empty()) {
            entry.faceMeshId = renderer->submitMesh(
                reinterpret_cast<const float*>(vertexData),
                numVerts,
                triIndices.data(),
                static_cast<int>(triIndices.size()),
                normalData,
                modelMatrix,
                diffuseColor,
                transparency
            );
        }
    }

    // Submit edge geometry (SoIndexedLineSet or subclass like SoBrepEdgeSet)
    auto* lineset = dynamic_cast<SoIndexedLineSet*>(
        findNodeOfType(root, SoIndexedLineSet::getClassTypeId())
    );
    if (lineset && lineset->coordIndex.getNum() > 0) {
        const int32_t* cindices = lineset->coordIndex.getValues(0);
        int numCIndices = lineset->coordIndex.getNum();

        std::vector<int32_t> lineIndices;
        lineIndices.reserve(numCIndices);
        for (int i = 0; i + 1 < numCIndices;) {
            if (cindices[i] >= 0 && cindices[i + 1] >= 0) {
                lineIndices.push_back(cindices[i]);
                lineIndices.push_back(cindices[i + 1]);
            }
            if (i + 2 < numCIndices && cindices[i + 2] >= 0) {
                i++;
            }
            else {
                i += 2;
                while (i < numCIndices && cindices[i] < 0) {
                    i++;
                }
            }
        }

        if (!lineIndices.empty()) {
            entry.lineMeshId = renderer->submitLines(
                reinterpret_cast<const float*>(vertexData),
                numVerts,
                lineIndices.data(),
                static_cast<int>(lineIndices.size()),
                modelMatrix,
                SbColor(0.0f, 0.0f, 0.0f),
                2.0f
            );
        }
    }

    // Submit point geometry (SoPointSet or subclass like SoBrepPointSet)
    auto* pointset = dynamic_cast<SoPointSet*>(findNodeOfType(root, SoPointSet::getClassTypeId()));
    if (pointset) {
        int numPoints = pointset->numPoints.getValue();
        if (numPoints > 0 && numPoints <= numVerts) {
            int startIdx = numVerts - numPoints;
            entry.pointMeshId = renderer->submitPoints(
                reinterpret_cast<const float*>(vertexData + startIdx),
                numPoints,
                modelMatrix,
                SbColor(0.0f, 0.0f, 0.0f),
                3.0f
            );
        }
    }
}
