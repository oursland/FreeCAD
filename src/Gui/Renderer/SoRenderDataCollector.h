// SPDX-License-Identifier: LGPL-2.1-or-later

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

#include <FCGlobal.h>
#include <Inventor/SbColor.h>
#include <Inventor/SbMatrix.h>

#include <cstdint>
#include <cstring>
#include <set>
#include <vector>

class SoNode;

namespace Gui
{

/// Data collected for each visible shape instance during SoGLRenderAction traversal.
/// Pointers (vertices, normals, coordIndices) point into Coin's own data and are
/// valid only for the duration of the current frame.
struct RenderItem
{
    /// The shape node that produced this item (SoBrepFaceSet, SoIndexedFaceSet, etc.)
    SoNode* shapeNode = nullptr;

    /// Geometry pointers (into Coin's data, valid for this frame only)
    const float* vertices = nullptr;  ///< xyz interleaved (SoCoordinateElement data)
    int numVertices = 0;
    const float* normals = nullptr;  ///< xyz interleaved (SoNormalElement data)
    int numNormals = 0;
    const int32_t* coordIndices = nullptr;
    int numCoordIndices = 0;
    std::vector<int32_t> ownedIndices;  ///< For non-indexed shapes that generate indices

    /// Resolved material (AFTER selection/highlight overrides applied by SoFCSelectionRoot)
    SbColor diffuseColor {0.8f, 0.8f, 0.8f};
    SbColor emissiveColor {0.0f, 0.0f, 0.0f};
    float transparency = 0.0f;

    /// Accumulated model matrix from traversal state
    SbMatrix modelMatrix;

    /// Selection state from SoFCSelectionContext (resolved during GLRender pass)
    int highlightIndex = -1;  ///< -1=none, INT_MAX=all, >=0=specific face
    SbColor highlightColor {0.0f, 0.0f, 0.0f};
    std::set<int> selectedIndices;  ///< empty=none, {-1}=all
    SbColor selectionColor {0.0f, 0.0f, 0.0f};

    /// Draw type
    enum Type
    {
        Triangles,
        Lines,
        Points
    } type = Triangles;

    float lineWidth = 1.0f;
    float pointSize = 3.0f;

    /// Unique key: hash of (shapeNode, modelMatrix) to distinguish linked instances
    uint64_t instanceKey() const
    {
        auto ptr = reinterpret_cast<uintptr_t>(shapeNode);
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

/// Collects RenderItem data during SoGLRenderAction traversal.
///
/// Shape nodes (SoBrepFaceSet, etc.) check for an active collector and emit
/// their resolved render data into it. This captures geometry, materials,
/// transforms, AND selection state — all resolved by Coin's normal traversal.
///
/// Usage:
///   SoRenderDataCollector collector;
///   SoRenderDataCollector::setActive(&collector);
///   glAction->apply(sceneRoot);
///   SoRenderDataCollector::setActive(nullptr);
///   // collector.items() now contains all visible shape data
class SoRenderDataCollector
{
public:
    /// If true, shape nodes should skip actual GL draw calls after emitting data.
    bool captureOnly = false;

    void addItem(RenderItem&& item)
    {
        items_.push_back(std::move(item));
    }

    const std::vector<RenderItem>& items() const
    {
        return items_;
    }

    void clear()
    {
        items_.clear();
    }

    /// Set the active collector. Shape nodes call getActive()
    /// during GLRender to check if they should emit data.
    /// Coin actions are single-threaded, so a plain static is safe.
    static GuiExport void setActive(SoRenderDataCollector* collector);

    /// Get the currently active collector (nullptr if none).
    static GuiExport SoRenderDataCollector* getActive();

private:
    std::vector<RenderItem> items_;
};

}  // namespace Gui
