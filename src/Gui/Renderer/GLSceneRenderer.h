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
#include "RenderQueue.h"
#include "ShaderProgram.h"

#include <memory>
#include <array>

namespace Gui
{

/// OpenGL 3.3 Core Profile implementation of SceneRenderer.
/// Maintains a flat render queue of VBO draw calls sorted for optimal
/// GPU throughput.  Replaces Coin3D's per-node recursive traversal.
class GLSceneRenderer: public SceneRenderer
{
public:
    GLSceneRenderer();
    ~GLSceneRenderer() override;

    // ---- Frame lifecycle ------------------------------------------------
    void beginFrame(
        const SbMatrix& viewMatrix,
        const SbMatrix& projMatrix,
        const SbViewportRegion& viewport
    ) override;
    void endFrame() override;

    // ---- Geometry submission --------------------------------------------
    MeshId submitMesh(
        const float* vertices,
        int numVerts,
        const int32_t* indices,
        int numIndices,
        const float* normals,
        const SbMatrix& transform,
        const SbColor& color,
        float transparency
    ) override;

    MeshId submitLines(
        const float* vertices,
        int numVerts,
        const int32_t* indices,
        int numIndices,
        const SbMatrix& transform,
        const SbColor& color,
        float lineWidth
    ) override;

    MeshId submitPoints(
        const float* vertices,
        int numVerts,
        const SbMatrix& transform,
        const SbColor& color,
        float pointSize
    ) override;

    // ---- Updates --------------------------------------------------------
    void updateTransform(MeshId id, const SbMatrix& transform) override;
    void updateMaterial(MeshId id, const SbColor& color, float transparency) override;
    void setVisible(MeshId id, bool visible) override;
    void remove(MeshId id) override;

    // ---- Selection/highlighting -----------------------------------------
    void setHighlight(MeshId id, int elementIndex, const SbColor& color) override;
    void clearHighlight(MeshId id) override;
    void setSelection(MeshId id, const std::vector<int>& elementIndices, const SbColor& color) override;
    void clearSelection(MeshId id) override;

    // ---- Invalidation ---------------------------------------------------
    void invalidate(MeshId id) override;

    /// Initialize GL resources (shaders, etc).  Must be called with a valid
    /// GL context active.  Returns false on failure.
    bool initialize();

    bool isInitialized() const
    {
        return initialized;
    }

private:
    void renderEntry(const DrawEntry& entry);

    bool initialized = false;
    MeshId nextId = 1;

    ShaderProgram meshShader;
    RenderQueue queue;

    // Uniform locations (cached after shader build)
    int uViewMatrix = -1;
    int uProjMatrix = -1;
    int uModelMatrix = -1;
    int uColor = -1;
    int uTransparency = -1;
    int uLightDir = -1;
    int uOverrideColor = -1;
    int uUseOverride = -1;

    // Current frame matrices
    std::array<float, 16> viewMatrix;
    std::array<float, 16> projMatrix;

    // Helper to create VAO/VBO/EBO for a draw entry
    DrawEntry createGLBuffers(
        DrawType type,
        const float* vertices,
        int numVerts,
        const float* normals,
        const int32_t* indices,
        int numIndices
    );
};

}  // namespace Gui
