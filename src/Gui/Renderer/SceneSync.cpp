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

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SbMatrix.h>

#include <Mod/Part/Gui/SoBrepFaceSet.h>
#include <Mod/Part/Gui/SoBrepEdgeSet.h>
#include <Mod/Part/Gui/SoBrepPointSet.h>
#include <Mod/Part/Gui/ViewProviderExt.h>

#include <Base/Console.h>

using namespace Gui;

SceneSync::SceneSync() = default;
SceneSync::~SceneSync() = default;

void SceneSync::sync(View3DInventorViewer* viewer, SceneRenderer* renderer)
{
    if (!viewer || !renderer) {
        return;
    }

    // Collect all currently active ViewProviders
    auto vpList = viewer->getViewProvidersOfType(PartGui::ViewProviderPartExt::getClassTypeId());

    // Track which VPs we've seen this frame
    std::map<ViewProvider*, bool> seen;

    for (auto* vp : vpList) {
        seen[vp] = true;

        // Check if VP is visible
        if (!vp->isVisible()) {
            // If we have entries for it, hide them
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

        // Check if this VP needs (re-)syncing
        uint32_t nodeId = vp->getRoot()->getNodeId();
        auto it = entries.find(vp);
        if (it != entries.end() && it->second.lastNodeId == nodeId) {
            // No change — ensure visible
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

        // Sync this VP
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
        kv.second.lastNodeId = 0;  // Force re-sync
    }
}

void SceneSync::syncViewProvider(ViewProvider* vp, SceneRenderer* renderer)
{
    auto* partVP = dynamic_cast<PartGui::ViewProviderPartExt*>(vp);
    if (!partVP) {
        return;
    }

    // Remove old entries if they exist
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

    // Get coordinate data
    SoCoordinate3* coords = partVP->coords;
    if (!coords || coords->point.getNum() == 0) {
        return;
    }

    const SbVec3f* vertexData = coords->point.getValues(0);
    int numVerts = coords->point.getNum();

    // Get transform
    SbMatrix modelMatrix;
    SoTransform* transform = dynamic_cast<SoTransform*>(partVP->getTransformNode());
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
    auto* geoVP = dynamic_cast<ViewProviderGeometryObject*>(vp);
    if (geoVP && geoVP->pcShapeMaterial) {
        if (geoVP->pcShapeMaterial->diffuseColor.getNum() > 0) {
            diffuseColor = geoVP->pcShapeMaterial->diffuseColor[0];
        }
        if (geoVP->pcShapeMaterial->transparency.getNum() > 0) {
            transparency = geoVP->pcShapeMaterial->transparency[0];
        }
    }

    // Submit face geometry
    SoBrepFaceSet* faceset = partVP->faceset;
    if (faceset && faceset->coordIndex.getNum() > 0) {
        // Convert Coin3D indexed face set indices (with -1 delimiters) to
        // flat triangle indices
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
            // Skip to next face (past the -1 delimiter)
            while (i < numCIndices && cindices[i] >= 0) {
                i++;
            }
            i++;  // skip the -1
        }

        // Get normals
        SoNormal* norm = partVP->norm;
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

    // Submit edge geometry
    SoBrepEdgeSet* lineset = partVP->lineset;
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
            // Skip to next segment
            if (i + 2 < numCIndices && cindices[i + 2] >= 0) {
                i++;  // next vertex in polyline
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
                SbColor(0.0f, 0.0f, 0.0f),  // edges are black
                2.0f
            );
        }
    }

    // Submit point geometry
    SoBrepPointSet* nodeset = partVP->nodeset;
    if (nodeset) {
        int startIdx = nodeset->startIndex.getValue();
        int numPoints = numVerts - startIdx;
        if (numPoints > 0) {
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
