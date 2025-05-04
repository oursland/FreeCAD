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

#include <Mod/Assembly/AssemblyGlobal.h>

#include <App/FeaturePython.h>
#include <App/Part.h>
#include <App/PropertyLinks.h>

namespace App
{
class PropertyXLinkSub;
}  // namespace App

namespace Base
{
class Placement;
class Rotation;
}  // namespace Base

namespace Assembly
{

class AssemblyLink;
class JointGroup;
class ViewGroup;
enum class JointType;

struct ObjRef
{
    App::DocumentObject* obj;
    App::PropertyXLinkSub* ref;
};

class AssemblyExport AssemblyObjectInterface: public App::Part
{
public:
    virtual ~AssemblyObjectInterface() = default;

    App::DocumentObjectExecReturn* execute() override = 0;

    static void recomputeJointPlacements(std::vector<App::DocumentObject*> joints);
    static void redrawJointPlacements(std::vector<App::DocumentObject*> joints);

    virtual int solve(bool enableRedo = false, bool updateJCS = true) = 0;
    virtual int generateSimulation(App::DocumentObject* sim) = 0;
    virtual int updateForFrame(size_t index, bool updateJCS = true) = 0;
    virtual size_t numberOfFrames() = 0;
    virtual void preDrag(std::vector<App::DocumentObject*> dragParts) = 0;
    virtual void doDragStep() = 0;
    virtual void postDrag() = 0;
    virtual void undoSolve() = 0;
    virtual void clearUndo() = 0;
    virtual void ensureIdentityPlacements() = 0;
    virtual JointGroup* getJointGroup() const = 0;
    virtual std::vector<App::DocumentObject*>
    getJoints(bool updateJCS = true, bool delBadJoints = false, bool subJoints = true) = 0;
    virtual std::vector<App::DocumentObject*> getGroundedJoints() = 0;
    virtual std::vector<App::DocumentObject*> getJointsOfObj(App::DocumentObject* obj) = 0;
    virtual std::vector<App::DocumentObject*> getJointsOfPart(App::DocumentObject* part) = 0;
    virtual App::DocumentObject* getJointOfPartConnectingToGround(App::DocumentObject* part,
                                                                  std::string& name) = 0;
    virtual bool isJointConnectingPartToGround(App::DocumentObject* joint,
                                               const char* partPropName) = 0;
    virtual bool isPartGrounded(App::DocumentObject* part) = 0;
    virtual bool isPartConnected(App::DocumentObject* part) = 0;
    virtual std::vector<ObjRef> getDownstreamParts(App::DocumentObject* part,
                                                   App::DocumentObject* joint = nullptr) = 0;
    virtual App::DocumentObject* getUpstreamMovingPart(App::DocumentObject* part,
                                                       App::DocumentObject*& joint,
                                                       std::string& name) = 0;
    virtual double getObjMass(App::DocumentObject* obj) = 0;
    virtual void
    setObjMasses(std::vector<std::pair<App::DocumentObject*, double>> objectMasses) = 0;
};

}  // namespace Assembly
