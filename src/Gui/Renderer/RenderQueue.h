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
#include <array>

namespace Gui
{

/// Type of geometry in a draw entry.
enum class DrawType : uint8_t
{
    Triangles,  ///< Face geometry (GL_TRIANGLES)
    Lines,      ///< Edge geometry (GL_LINES)
    Points      ///< Vertex geometry (GL_POINTS)
};

/// A single entry in the render queue.  Contains everything needed to
/// issue one draw call: GPU buffer handles, transform, material, and
/// selection state.
struct DrawEntry
{
    uint32_t meshId = 0;

    // GL object handles
    uint32_t vao = 0;
    uint32_t vbo = 0;     // vertex + normal interleaved
    uint32_t ebo = 0;     // element (index) buffer
    int numElements = 0;  // number of indices (or vertices for points)

    DrawType type = DrawType::Triangles;
    bool visible = true;

    // Transform (4x4 column-major)
    std::array<float, 16> modelMatrix;

    // Material
    std::array<float, 3> color = {0.8f, 0.8f, 0.8f};
    float transparency = 0.0f;
    float lineWidth = 1.0f;
    float pointSize = 3.0f;

    // Selection state
    int highlightElement = -1;  // -1 = no highlight
    std::array<float, 3> highlightColor = {0.0f, 1.0f, 0.0f};
    std::vector<int> selectedElements;
    std::array<float, 3> selectionColor = {0.0f, 0.5f, 0.0f};

    /// Sort key for draw order.  Lower = drawn first.
    /// Opaque before transparent, faces before edges before points.
    uint64_t sortKey() const;
};

/// Flat list of draw calls, sorted for optimal rendering.
class RenderQueue
{
public:
    /// Add or update an entry.  If meshId already exists, replaces it.
    void addOrUpdate(const DrawEntry& entry);

    /// Remove an entry by meshId.
    void remove(uint32_t meshId);

    /// Find an entry by meshId.  Returns nullptr if not found.
    DrawEntry* find(uint32_t meshId);
    const DrawEntry* find(uint32_t meshId) const;

    /// Sort entries for optimal draw order.
    void sort();

    /// Access sorted entries for rendering.
    const std::vector<DrawEntry>& entries() const
    {
        return queue;
    }
    std::vector<DrawEntry>& entries()
    {
        return queue;
    }

    void clear()
    {
        queue.clear();
    }
    size_t size() const
    {
        return queue.size();
    }

private:
    std::vector<DrawEntry> queue;
    bool needsSort = false;
};

}  // namespace Gui
