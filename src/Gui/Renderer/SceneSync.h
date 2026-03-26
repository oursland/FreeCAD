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

#pragma once

#include "SceneRenderer.h"
#include "SoRenderDataCollector.h"

#include <cstdint>
#include <map>
#include <vector>

class SoNode;

namespace Gui
{

class View3DInventorViewer;

/// Synchronizes render data from the Coin3D scene graph to a SceneRenderer.
///
/// Uses SoRenderDataCollector to intercept data during SoGLRenderAction
/// traversal. Shape nodes (SoBrepFaceSet, etc.) emit RenderItems into the
/// collector, which captures geometry, transforms, materials, AND selection
/// state — all resolved by Coin's normal traversal with SoFCSelectionRoot
/// color overrides already applied.
class SceneSync
{
public:
    SceneSync();
    ~SceneSync();

    /// Synchronize the scene graph to the renderer.
    /// Runs a collector-enabled SoGLRenderAction pass, then processes
    /// the collected RenderItems: new items are submitted, changed items
    /// updated, removed items deleted, and selection state applied.
    void sync(View3DInventorViewer* viewer, SceneRenderer* renderer);

    /// Force a full re-sync on the next call to sync().
    void invalidateAll(SceneRenderer* renderer);

    /// Mark the scene as dirty — next sync() will re-run the collector pass.
    /// Call this when visibility or structural changes occur.
    void markDirty()
    {
        needsFullSync = true;
    }

    /// Update selection/preselection highlighting without re-traversing.
    /// Queries the Selection singleton and applies highlight colors to
    /// matching mesh entries based on the shape node pointer.
    void updateHighlighting(View3DInventorViewer* viewer, SceneRenderer* renderer);

private:
    void processItems(const std::vector<RenderItem>& items, SceneRenderer* renderer);

    struct MeshEntry
    {
        SceneRenderer::MeshId meshId = SceneRenderer::InvalidMesh;
        RenderItem::Type type = RenderItem::Triangles;
        SoNode* shapeNode = nullptr;  ///< The shape node for selection matching
    };

    std::map<uint64_t, MeshEntry> meshEntries;
    /// Reverse lookup: shape node → list of mesh IDs (multiple for Assembly Links)
    std::map<SoNode*, std::vector<SceneRenderer::MeshId>> nodeToMeshIds;
    bool needsFullSync = true;
    uint32_t lastObjectGroupNodeId = 0;
};

}  // namespace Gui
