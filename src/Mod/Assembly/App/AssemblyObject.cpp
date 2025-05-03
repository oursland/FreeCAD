/***************************************************************************
 *   Copyright (c) 2025 Jacob Oursland <jacob.oursland[at]gmail.com>       *
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

#include "AssemblyObject.h"

#include <utility>
#include "AssemblyObjectPy.h"
#include "OndselAssemblyObject.h"

FC_LOG_LEVEL_INIT("Assembly", true, true, true)

using namespace Assembly;

PROPERTY_SOURCE(Assembly::AssemblyObject, App::Part)

// AssemblyObject* AssemblyObject::instance = nullptr;

// AssemblyObject::AssemblyObject(AssemblyObject* instance)
AssemblyObject::AssemblyObject()
    : instance(new Assembly::OndselAssemblyObject())
{
    // if (instance == nullptr) {
    //     instance = new Assembly::OndselAssemblyObject();
    // }
    // AssemblyObject::instance = instance;
}

AssemblyObject::~AssemblyObject()
{
    delete AssemblyObject::instance;
}

PyObject* AssemblyObject::getPyObject()
{
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new AssemblyObjectPy(this), true);
    }
    return Py::new_reference_to(PythonObject);
}

App::DocumentObjectExecReturn* AssemblyObject::execute()
{
    App::DocumentObjectExecReturn* ret = App::Part::execute();

    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Assembly");
    if (hGrp->GetBool("SolveOnRecompute", true)) {
        solve();
    }
    return ret;
}

int AssemblyObject::solve(bool enableRedo, bool updateJCS)
{
    return AssemblyObject::instance->solve(enableRedo, updateJCS);
}

int AssemblyObject::generateSimulation(App::DocumentObject* sim)
{
    return AssemblyObject::instance->generateSimulation(sim);
}

int AssemblyObject::updateForFrame(size_t index, bool updateJCS)
{
    return AssemblyObject::instance->updateForFrame(index, updateJCS);
}

size_t AssemblyObject::numberOfFrames()
{
    return AssemblyObject::instance->numberOfFrames();
}

void AssemblyObject::preDrag(std::vector<App::DocumentObject*> dragParts)
{
    AssemblyObject::instance->preDrag(std::move(dragParts));
}

void AssemblyObject::doDragStep()
{
    AssemblyObject::instance->doDragStep();
}

void AssemblyObject::postDrag()
{
    AssemblyObject::instance->postDrag();
}

void AssemblyObject::undoSolve()
{
    AssemblyObject::instance->undoSolve();
}

void AssemblyObject::clearUndo()
{
    AssemblyObject::instance->clearUndo();
}

void AssemblyObject::recomputeJointPlacements(std::vector<App::DocumentObject*> joints)
{
    OndselAssemblyObject::recomputeJointPlacements(std::move(joints));
}

void AssemblyObject::redrawJointPlacements(std::vector<App::DocumentObject*> joints)
{
    OndselAssemblyObject::redrawJointPlacements(std::move(joints));
}

void AssemblyObject::ensureIdentityPlacements()
{
    AssemblyObject::instance->ensureIdentityPlacements();
}

JointGroup* AssemblyObject::getJointGroup() const
{
    return AssemblyObject::instance->getJointGroup();
}

std::vector<App::DocumentObject*>
AssemblyObject::getJoints(bool updateJCS, bool delBadJoints, bool subJoints)
{
    return AssemblyObject::instance->getJoints(updateJCS, delBadJoints, subJoints);
}

std::vector<App::DocumentObject*> AssemblyObject::getGroundedJoints()
{
    return AssemblyObject::instance->getGroundedJoints();
}

std::vector<App::DocumentObject*> AssemblyObject::getJointsOfObj(App::DocumentObject* obj)
{
    return AssemblyObject::instance->getJointsOfObj(obj);
}

std::vector<App::DocumentObject*> AssemblyObject::getJointsOfPart(App::DocumentObject* part)
{
    return AssemblyObject::instance->getJointsOfPart(part);
}

App::DocumentObject* AssemblyObject::getJointOfPartConnectingToGround(App::DocumentObject* part,
                                                                      std::string& name)
{
    return AssemblyObject::instance->getJointOfPartConnectingToGround(part, name);
}

bool AssemblyObject::isJointConnectingPartToGround(App::DocumentObject* joint,
                                                   const char* partPropName)
{
    return AssemblyObject::instance->isJointConnectingPartToGround(joint, partPropName);
}

bool AssemblyObject::isPartGrounded(App::DocumentObject* part)
{
    return AssemblyObject::instance->isPartGrounded(part);
}

bool AssemblyObject::isPartConnected(App::DocumentObject* part)
{
    return AssemblyObject::instance->isPartConnected(part);
}

std::vector<ObjRef> AssemblyObject::getDownstreamParts(App::DocumentObject* part,
                                                       App::DocumentObject* joint)
{
    return AssemblyObject::instance->getDownstreamParts(part, joint);
}

App::DocumentObject* AssemblyObject::getUpstreamMovingPart(App::DocumentObject* part,
                                                           App::DocumentObject*& joint,
                                                           std::string& name)
{
    return AssemblyObject::instance->getUpstreamMovingPart(part, joint, name);
}

double AssemblyObject::getObjMass(App::DocumentObject* obj)
{
    return AssemblyObject::instance->getObjMass(obj);
}

void AssemblyObject::setObjMasses(std::vector<std::pair<App::DocumentObject*, double>> objectMasses)
{
    AssemblyObject::instance->setObjMasses(std::move(objectMasses));
}
