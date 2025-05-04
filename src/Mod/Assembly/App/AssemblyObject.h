// SPDX-License-Identifier: LGPL-2.1-or-later
/****************************************************************************
 *                                                                          *
 *   Copyright (c) 2025 Jacob Oursland <jacob.oursland[at]gmail.com>        *
 *   Copyright (c) 2023 Ondsel <development@ondsel.com>                     *
 *                                                                          *
 *   This file is part of FreeCAD.                                          *
 *                                                                          *
 *   FreeCAD is free software: you can redistribute it and/or modify it     *
 *   under the terms of the GNU Lesser General Public License as            *
 *   published by the Free Software Foundation, either version 2.1 of the   *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   FreeCAD is distributed in the hope that it will be useful, but         *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of             *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU       *
 *   Lesser General Public License for more details.                        *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with FreeCAD. If not, see                                *
 *   <https://www.gnu.org/licenses/>.                                       *
 *                                                                          *
 ***************************************************************************/

#pragma once

#include "AssemblyObjectInterface.h"

#include <Mod/Assembly/AssemblyGlobal.h>

#include <App/FeaturePython.h>
#include <App/Part.h>
#include <App/PropertyLinks.h>

namespace Assembly
{

class AssemblyExport AssemblyObject: public AssemblyObjectInterface
{
    PROPERTY_HEADER_WITH_OVERRIDE(Assembly::AssemblyObject);

public:
    // explicit AssemblyObject(AssemblyObject *instance = nullptr);
    AssemblyObject();
    ~AssemblyObject() override;

    PyObject* getPyObject() override;

    const char* getViewProviderName() const override
    {
        return "AssemblyGui::ViewProviderAssembly";
    }

    App::DocumentObjectExecReturn* execute() override;

    /* Solve the assembly. It will update first the joints, solve, update placements of the parts
    and redraw the joints Args : enableRedo : This store initial positions to enable undo while
    being in an active transaction (joint creation).*/
    int solve(bool enableRedo = false, bool updateJCS = true) override;
    int generateSimulation(App::DocumentObject* sim) override;
    int updateForFrame(size_t index, bool updateJCS = true) override;
    size_t numberOfFrames() override;
    void preDrag(std::vector<App::DocumentObject*> dragParts) override;
    void doDragStep() override;
    void postDrag() override;
    void undoSolve() override;
    void clearUndo() override;

    static void recomputeJointPlacements(std::vector<App::DocumentObject*> joints);
    static void redrawJointPlacements(std::vector<App::DocumentObject*> joints);

    // This makes sure that LinkGroups or sub-assemblies have identity placements.
    void ensureIdentityPlacements() override;

    JointGroup* getJointGroup() const override;

    std::vector<App::DocumentObject*>
    getJoints(bool updateJCS = true, bool delBadJoints = false, bool subJoints = true) override;
    std::vector<App::DocumentObject*> getGroundedJoints() override;
    std::vector<App::DocumentObject*> getJointsOfObj(App::DocumentObject* obj) override;
    std::vector<App::DocumentObject*> getJointsOfPart(App::DocumentObject* part) override;
    App::DocumentObject* getJointOfPartConnectingToGround(App::DocumentObject* part,
                                                          std::string& name) override;

    bool isJointConnectingPartToGround(App::DocumentObject* joint,
                                       const char* partPropName) override;

    bool isPartGrounded(App::DocumentObject* part) override;
    bool isPartConnected(App::DocumentObject* part) override;

    std::vector<ObjRef> getDownstreamParts(App::DocumentObject* part,
                                           App::DocumentObject* joint = nullptr) override;
    App::DocumentObject* getUpstreamMovingPart(App::DocumentObject* part,
                                               App::DocumentObject*& joint,
                                               std::string& name) override;

    double getObjMass(App::DocumentObject* obj) override;
    void setObjMasses(std::vector<std::pair<App::DocumentObject*, double>> objectMasses) override;

private:
    AssemblyObjectInterface* instance;
};

}  // namespace Assembly
