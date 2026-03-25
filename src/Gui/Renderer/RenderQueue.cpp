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

#include "RenderQueue.h"
#include <algorithm>

using namespace Gui;

uint64_t DrawEntry::sortKey() const
{
    // Sort order:
    //   Bit 63:    1 = transparent, 0 = opaque  (opaque first)
    //   Bit 60-62: draw type (Triangles=0, Lines=1, Points=2)
    //   Bit 0-31:  mesh ID (for stable ordering)
    uint64_t key = 0;
    if (transparency > 0.001f) {
        key |= (1ULL << 63);
    }
    key |= (static_cast<uint64_t>(type) << 60);
    key |= static_cast<uint64_t>(meshId);
    return key;
}

void RenderQueue::addOrUpdate(const DrawEntry& entry)
{
    for (auto& e : queue) {
        if (e.meshId == entry.meshId) {
            e = entry;
            needsSort = true;
            return;
        }
    }
    queue.push_back(entry);
    needsSort = true;
}

void RenderQueue::remove(uint32_t meshId)
{
    queue.erase(
        std::remove_if(
            queue.begin(),
            queue.end(),
            [meshId](const DrawEntry& e) { return e.meshId == meshId; }
        ),
        queue.end()
    );
}

DrawEntry* RenderQueue::find(uint32_t meshId)
{
    for (auto& e : queue) {
        if (e.meshId == meshId) {
            return &e;
        }
    }
    return nullptr;
}

const DrawEntry* RenderQueue::find(uint32_t meshId) const
{
    for (const auto& e : queue) {
        if (e.meshId == meshId) {
            return &e;
        }
    }
    return nullptr;
}

void RenderQueue::sort()
{
    if (!needsSort) {
        return;
    }
    std::sort(queue.begin(), queue.end(), [](const DrawEntry& a, const DrawEntry& b) {
        return a.sortKey() < b.sortKey();
    });
    needsSort = false;
}
