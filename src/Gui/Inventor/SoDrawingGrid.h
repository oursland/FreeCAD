// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2010 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#pragma once

#include <Inventor/SbVec2s.h>
#include <Inventor/nodes/SoShape.h>
#include <FCGlobal.h>

class SoLineSet;
class SoSeparator;
class SoState;
class SoVertexProperty;

namespace Gui
{
namespace Inventor
{

class GuiExport SoDrawingGrid: public SoShape
{
    using inherited = SoShape;

    SO_NODE_HEADER(SoDrawingGrid);

public:
    static void initClass();
    SoDrawingGrid();

public:
    void GLRender(SoGLRenderAction* action) override;
    void GLRenderBelowPath(SoGLRenderAction* action) override;
    void GLRenderInPath(SoGLRenderAction* action) override;
    void GLRenderOffPath(SoGLRenderAction* action) override;
    void computeBBox(SoAction* action, SbBox3f& box, SbVec3f& center) override;
    void generatePrimitives(SoAction* action) override;

private:
    void renderGrid(SoGLRenderAction* action);
    void ensureGeometry(SoState* state);
    // Force using the reference count mechanism.
    ~SoDrawingGrid() override;

private:
    SoSeparator* m_Root {nullptr};
    SoVertexProperty* m_VertexProperty {nullptr};
    SoLineSet* m_LineSet {nullptr};
    SbVec2s m_CachedViewportSize {0, 0};
};

}  // namespace Inventor

}  // namespace Gui
