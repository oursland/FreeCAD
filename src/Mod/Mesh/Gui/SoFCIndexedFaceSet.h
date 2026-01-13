// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2009 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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

#include <Inventor/engines/SoSubEngine.h>
#include <Inventor/fields/SoMFColor.h>
#include <Inventor/fields/SoSFBool.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#ifndef MESH_GLOBAL_H
# include <Mod/Mesh/MeshGlobal.h>
#endif

namespace MeshGui
{

// NOLINTBEGIN
class MeshRenderer
{
public:
    static bool shouldRenderDirectly(bool);
};

/**
 * class SoFCMaterialEngine
 * \brief The SoFCMaterialEngine class is used to notify an
 * SoFCIndexedFaceSet node about material changes.
 *
 * @author Werner Mayer
 */
class MeshGuiExport SoFCMaterialEngine: public SoEngine
{
    SO_ENGINE_HEADER(SoFCMaterialEngine);

public:
    SoFCMaterialEngine();
    static void initClass();

    SoMFColor diffuseColor;
    SoEngineOutput trigger;

private:
    ~SoFCMaterialEngine() override;
    void evaluate() override;
    void inputChanged(SoField*) override;
};

/**
 * class SoFCIndexedFaceSet
 * \brief The SoFCIndexedFaceSet class is designed to optimize redrawing a mesh
 * during user interaction.
 *
 * @author Werner Mayer
 */
class MeshGuiExport SoFCIndexedFaceSet: public SoIndexedFaceSet
{
    using inherited = SoIndexedFaceSet;

    SO_NODE_HEADER(SoFCIndexedFaceSet);

public:
    static void initClass();
    SoFCIndexedFaceSet();

    SoSFBool updateGLArray;
    unsigned int renderTriangleLimit;

    void invalidate();

protected:
    // Force using the reference count mechanism.
    ~SoFCIndexedFaceSet() override = default;
    void GLRender(SoGLRenderAction* action) override;

    void doAction(SoAction* action) override;

};
// NOLINTEND

}  // namespace MeshGui
