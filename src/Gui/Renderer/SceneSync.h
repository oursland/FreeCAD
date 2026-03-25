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

#include <map>
#include <set>
#include <cstdint>

namespace Gui
{

class View3DInventorViewer;

/// Synchronizes render data from the Coin3D scene graph to a SceneRenderer.
///
/// Uses SoCallbackAction to traverse the visible scene graph (respecting
/// SoSwitch nodes for Links, display modes, and hidden objects). Extracts
/// geometry, transforms, and materials from the traversal state and submits
/// them to the renderer.
class SceneSync
{
public:
    SceneSync();
    ~SceneSync();

    /// Synchronize the scene graph to the renderer.
    /// Call this before beginFrame() each frame.  Only re-submits
    /// geometry on first call or after invalidateAll().
    void sync(View3DInventorViewer* viewer, SceneRenderer* renderer);

    /// Force a full re-sync on the next call to sync().
    void invalidateAll();

private:
    /// Per-faceset tracking data
    struct SyncEntry
    {
        SceneRenderer::MeshId faceMeshId = SceneRenderer::InvalidMesh;
    };

    std::map<uint64_t, SyncEntry> meshEntries;  // keyed by instance key (node ptr + matrix hash)
};

}  // namespace Gui
