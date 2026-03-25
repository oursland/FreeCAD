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

#include <cstdint>
#include <vector>

class SbMatrix;
class SbColor;
class SbViewportRegion;

namespace Gui
{

/// Abstract rendering interface that decouples FreeCAD's scene graph from
/// the rendering backend.  Phase 1 implements an OpenGL 3.3 backend;
/// future backends (Hydra/Storm, Metal, Vulkan) implement the same interface.
///
/// Geometry is submitted once (when the scene graph changes) and drawn
/// every frame.  The renderer maintains an internal draw list sorted for
/// optimal GPU throughput.
class SceneRenderer
{
public:
    using MeshId = uint32_t;
    static constexpr MeshId InvalidMesh = 0;

    virtual ~SceneRenderer() = default;

    // ---- Frame lifecycle ------------------------------------------------

    /// Prepare for a new frame.  Sets camera matrices and viewport.
    virtual void beginFrame(
        const SbMatrix& viewMatrix,
        const SbMatrix& projMatrix,
        const SbViewportRegion& viewport
    ) = 0;

    /// Execute all queued draw calls and present.
    virtual void endFrame() = 0;

    // ---- Geometry submission --------------------------------------------
    // Called when the scene graph changes, NOT every frame.

    /// Submit a triangle mesh (faces).  Returns a handle for later updates.
    virtual MeshId submitMesh(
        const float* vertices,
        int numVerts,
        const int32_t* indices,
        int numIndices,
        const float* normals,
        const SbMatrix& transform,
        const SbColor& color,
        float transparency
    ) = 0;

    /// Submit a line set (edges).
    virtual MeshId submitLines(
        const float* vertices,
        int numVerts,
        const int32_t* indices,
        int numIndices,
        const SbMatrix& transform,
        const SbColor& color,
        float lineWidth
    ) = 0;

    /// Submit a point set (vertices).
    virtual MeshId submitPoints(
        const float* vertices,
        int numVerts,
        const SbMatrix& transform,
        const SbColor& color,
        float pointSize
    ) = 0;

    // ---- Updates (when transform, material, or visibility changes) ------

    virtual void updateTransform(MeshId id, const SbMatrix& transform) = 0;
    virtual void updateMaterial(MeshId id, const SbColor& color, float transparency) = 0;
    virtual void setVisible(MeshId id, bool visible) = 0;
    virtual void remove(MeshId id) = 0;

    // ---- Selection and highlighting (per-element within a mesh) ---------

    /// Highlight a single element (face/edge/vertex) for preselection.
    virtual void setHighlight(MeshId id, int elementIndex, const SbColor& color) = 0;
    virtual void clearHighlight(MeshId id) = 0;

    /// Select multiple elements.
    virtual void setSelection(MeshId id, const std::vector<int>& elementIndices, const SbColor& color)
        = 0;
    virtual void clearSelection(MeshId id) = 0;

    // ---- Invalidation ---------------------------------------------------

    /// Geometry data changed — re-upload VBOs on next frame.
    virtual void invalidate(MeshId id) = 0;
};

}  // namespace Gui
