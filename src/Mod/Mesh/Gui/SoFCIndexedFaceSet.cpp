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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/bundles/SoMaterialBundle.h>
#include <Inventor/elements/SoViewportRegionElement.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoSeparator.h>

#include <QImage>

#include <Gui/SoFCOffscreenRenderer.h>
#include <Gui/Selection/SoFCSelectionAction.h>

#include "SoFCIndexedFaceSet.h"

using namespace MeshGui;

bool MeshRenderer::shouldRenderDirectly(bool)
{
    // Keep using the SoCoordinate3 + SoFCIndexedFaceSet path for now.
    // The legacy "direct rendering" path still uses raw OpenGL elsewhere.
    return false;
}

// ----------------------------------------------------------------------------

SO_ENGINE_SOURCE(SoFCMaterialEngine)

SoFCMaterialEngine::SoFCMaterialEngine()
{
    SO_ENGINE_CONSTRUCTOR(SoFCMaterialEngine);

    SO_ENGINE_ADD_INPUT(diffuseColor, (SbColor(0.0, 0.0, 0.0)));
    SO_ENGINE_ADD_OUTPUT(trigger, SoSFBool);
}

SoFCMaterialEngine::~SoFCMaterialEngine() = default;

void SoFCMaterialEngine::initClass()
{
    SO_ENGINE_INIT_CLASS(SoFCMaterialEngine, SoEngine, "Engine");
}

void SoFCMaterialEngine::inputChanged(SoField*)
{
    SO_ENGINE_OUTPUT(trigger, SoSFBool, setValue(true));
}

void SoFCMaterialEngine::evaluate()
{
    // do nothing here
}

// ----------------------------------------------------------------------------

SO_NODE_SOURCE(SoFCIndexedFaceSet)

void SoFCIndexedFaceSet::initClass()
{
    SO_NODE_INIT_CLASS(SoFCIndexedFaceSet, SoIndexedFaceSet, "IndexedFaceSet");
}

SoFCIndexedFaceSet::SoFCIndexedFaceSet()
    : renderTriangleLimit(std::numeric_limits<unsigned>::max())
{
    SO_NODE_CONSTRUCTOR(SoFCIndexedFaceSet);
    SO_NODE_ADD_FIELD(updateGLArray, (false));
    updateGLArray.setFieldType(SoField::EVENTOUT_FIELD);
    setName(SoFCIndexedFaceSet::getClassTypeId().getName());
}

void SoFCIndexedFaceSet::GLRender(SoGLRenderAction* action)
{
    if (this->coordIndex.getNum() < 3) {
        return;
    }

    if (!this->shouldGLRender(action)) {
        // Transparency is handled inside 'shouldGLRender' but the base class
        // can miss resetting the blending mode in some cases.
        // Calling SoMaterialBundle::sendFirst fixes the problem.
        SoMaterialBundle mb(action);
        mb.sendFirst();
        return;
    }

    if (updateGLArray.getValue()) {
        // Previously used to trigger regeneration of raw OpenGL buffers.
        // Keep the field for compatibility, but just invalidate Coin caches.
        updateGLArray.setValue(false);
        this->touch();
    }

    inherited::GLRender(action);
}

void SoFCIndexedFaceSet::invalidate()
{
    updateGLArray.setValue(true);
    this->touch();
}

void SoFCIndexedFaceSet::doAction(SoAction* action)
{
    auto applySelection = [this](SoAction* action, Gui::SoGLSelectAction* doaction, SoNode* node) {
        SoCamera* camera = nullptr;
        SoCoordinate3* coord = nullptr;

        {
            SoSearchAction sa;
            sa.setInterest(SoSearchAction::FIRST);
            sa.setSearchingAll(false);
            sa.setType(SoCamera::getClassTypeId(), 1);
            sa.apply(node);
            SoPath* path = sa.getPath();
            if (path) {
                SoNode* found = path->getNodeFromTail(0);
                if (found && found->getTypeId().isDerivedFrom(SoCamera::getClassTypeId())) {
                    camera = static_cast<SoCamera*>(found);
                }
            }
        }

        {
            SoSearchAction sa;
            sa.setInterest(SoSearchAction::FIRST);
            sa.setSearchingAll(false);
            sa.setType(SoCoordinate3::getClassTypeId(), 1);
            sa.apply(node);
            SoPath* path = sa.getPath();
            if (path) {
                SoNode* found = path->getNodeFromTail(0);
                if (found && found->getTypeId().isDerivedFrom(SoCoordinate3::getClassTypeId())) {
                    coord = static_cast<SoCoordinate3*>(found);
                }
            }
        }

        if (!camera || !coord) {
            return;
        }

        const uint32_t numFaces = static_cast<uint32_t>(this->coordIndex.getNum() / 4);
        if (numFaces == 0 || numFaces >= 0x00ffffffU) {
            return;
        }

        const SbViewportRegion& fullVp = SoViewportRegionElement::get(action->getState());
        const SbViewportRegion& selVp = doaction->getViewportRegion();

        const SbVec2s fullOrigin = fullVp.getViewportOriginPixels();
        const SbVec2s selCenter = selVp.getViewportOriginPixels();
        const SbVec2s selSize = selVp.getViewportSizePixels();

        const int selW = selSize[0];
        const int selH = selSize[1];
        if (selW <= 0 || selH <= 0) {
            return;
        }

        const int centerX = selCenter[0] - fullOrigin[0];
        const int centerY = selCenter[1] - fullOrigin[1];

        const int minX = centerX - (selW / 2);
        const int minY = centerY - (selH / 2);
        const int maxX = minX + selW - 1;
        const int maxY = minY + selH - 1;

        auto root = new SoSeparator;
        root->ref();
        root->addChild(camera);

        auto lm = new SoLightModel();
        lm->model = SoLightModel::BASE_COLOR;
        root->addChild(lm);

        auto mat = new SoMaterial();
        mat->transparency = 0.0F;
        mat->diffuseColor.setNum(numFaces);
        SbColor* diffcol = mat->diffuseColor.startEditing();
        for (uint32_t i = 0; i < numFaces; i++) {
            const uint32_t id = i + 1;
            const float r = static_cast<float>((id >> 16U) & 0xffU) / 255.0F;
            const float g = static_cast<float>((id >> 8U) & 0xffU) / 255.0F;
            const float b = static_cast<float>(id & 0xffU) / 255.0F;
            diffcol[i].setValue(r, g, b);
        }
        mat->diffuseColor.finishEditing();

        auto bind = new SoMaterialBinding();
        bind->value = SoMaterialBinding::PER_FACE;

        root->addChild(mat);
        root->addChild(bind);
        root->addChild(coord);
        root->addChild(this);

        Gui::SoQtOffscreenRenderer renderer(fullVp);
        renderer.setBackgroundColor(SbColor4f(0.0F, 0.0F, 0.0F, 0.0F));

        QImage img;
        const SbBool rendered = renderer.render(root);
        if (rendered) {
            renderer.writeToImage(img);
        }
        root->unref();

        if (!rendered || img.isNull()) {
            return;
        }

        const int imgW = img.width();
        const int imgH = img.height();
        if (imgW <= 0 || imgH <= 0) {
            return;
        }

        const auto clamp = [](int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); };

        const int x0 = clamp(minX, 0, imgW - 1);
        const int x1 = clamp(maxX, 0, imgW - 1);
        const int y0 = clamp(minY, 0, imgH - 1);
        const int y1 = clamp(maxY, 0, imgH - 1);

        std::vector<unsigned long> picked;
        picked.reserve(256);

        for (int y = y0; y <= y1; y++) {
            const int imgY = (imgH - 1) - y;
            for (int x = x0; x <= x1; x++) {
                const QRgb pixel = img.pixel(x, imgY);
                const uint32_t packed = (static_cast<uint32_t>(qRed(pixel)) << 16U)
                    | (static_cast<uint32_t>(qGreen(pixel)) << 8U)
                    | static_cast<uint32_t>(qBlue(pixel));
                if (packed == 0) {
                    continue;
                }
                const uint32_t face = packed - 1;
                if (face < numFaces) {
                    picked.push_back(static_cast<unsigned long>(face));
                }
            }
        }

        std::sort(picked.begin(), picked.end());
        picked.erase(std::unique(picked.begin(), picked.end()), picked.end());
        doaction->indices.insert(doaction->indices.end(), picked.begin(), picked.end());
        doaction->setHandled();
    };

    if (action->getTypeId() == Gui::SoGLSelectAction::getClassTypeId()) {
        SoNode* node = action->getNodeAppliedTo();
        if (!node) {
            return;
        }
        applySelection(action, static_cast<Gui::SoGLSelectAction*>(action), node);
    }

    inherited::doAction(action);
}

