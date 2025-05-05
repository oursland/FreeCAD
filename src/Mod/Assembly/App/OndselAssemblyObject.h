// SPDX-License-Identifier: LGPL-2.1-or-later
/****************************************************************************
 *                                                                          *
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

#include "App/DocumentObject.h"
#include "AssemblyObjectInterface.h"

#include <Mod/Assembly/AssemblyGlobal.h>

#include <App/FeaturePython.h>
#include <App/Part.h>
#include <App/PropertyLinks.h>

#include <OndselSolver/enum.h>

namespace MbD
{
class ASMTPart;
class ASMTAssembly;
class ASMTJoint;
class ASMTMarker;
class ASMTPart;
}  // namespace MbD

namespace Assembly
{

class AssemblyExport OndselAssemblyObject: public AssemblyObjectInterface
{
public:
    OndselAssemblyObject();
    ~OndselAssemblyObject() override;

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
    void savePlacementsForUndo();
    void undoSolve() override;
    void clearUndo() override;

    void exportAsASMT(std::string fileName);

    Base::Placement getMbdPlacement(std::shared_ptr<MbD::ASMTPart> mbdPart);
    bool validateNewPlacements();
    void setNewPlacements();
    static void recomputeJointPlacements(std::vector<App::DocumentObject*> joints);
    static void redrawJointPlacements(std::vector<App::DocumentObject*> joints);
    static void redrawJointPlacement(App::DocumentObject* joint);

    // This makes sure that LinkGroups or sub-assemblies have identity placements.
    void ensureIdentityPlacements() override;

    // Ondsel Solver interface
    std::shared_ptr<MbD::ASMTAssembly> makeMbdAssembly();
    void create_mbdSimulationParameters(App::DocumentObject* sim);
    std::shared_ptr<MbD::ASMTPart>
    makeMbdPart(std::string& name, Base::Placement plc = Base::Placement(), double mass = 1.0);
    std::shared_ptr<MbD::ASMTPart> getMbDPart(App::DocumentObject* obj);
    // To help the solver, during dragging, we are bundling parts connected by a fixed joint.
    // So several assembly components are bundled in a single ASMTPart.
    // So we need to store the plc of each bundled object relative to the bundle origin (first obj
    // of objectPartMap).
    struct MbDPartData
    {
        std::shared_ptr<MbD::ASMTPart> part;
        Base::Placement offsetPlc;  // This is the offset within the bundled parts
    };
    MbDPartData getMbDData(App::DocumentObject* part);
    std::shared_ptr<MbD::ASMTMarker> makeMbdMarker(std::string& name, Base::Placement& plc);
    std::vector<std::shared_ptr<MbD::ASMTJoint>> makeMbdJoint(App::DocumentObject* joint);
    std::shared_ptr<MbD::ASMTJoint> makeMbdJointOfType(App::DocumentObject* joint,
                                                       JointType jointType);
    std::shared_ptr<MbD::ASMTJoint> makeMbdJointDistance(App::DocumentObject* joint);
    std::string handleOneSideOfJoint(App::DocumentObject* joint,
                                     const char* propRefName,
                                     const char* propPlcName);
    void getRackPinionMarkers(App::DocumentObject* joint,
                              std::string& markerNameI,
                              std::string& markerNameJ);
    int slidingPartIndex(App::DocumentObject* joint);

    void jointParts(std::vector<App::DocumentObject*> joints);
    JointGroup* getJointGroup() const override;
    ViewGroup* getExplodedViewGroup() const;
    template<typename T>
    T* getGroup();

    std::vector<App::DocumentObject*>
    getJoints(bool updateJCS = true, bool delBadJoints = false, bool subJoints = true) override;
    std::vector<App::DocumentObject*> getGroundedJoints() override;
    std::vector<App::DocumentObject*> getJointsOfObj(App::DocumentObject* obj) override;
    std::vector<App::DocumentObject*> getJointsOfPart(App::DocumentObject* part) override;
    App::DocumentObject* getJointOfPartConnectingToGround(App::DocumentObject* part,
                                                          std::string& name) override;
    std::unordered_set<App::DocumentObject*> getGroundedParts();
    std::unordered_set<App::DocumentObject*> fixGroundedParts();
    void fixGroundedPart(App::DocumentObject* obj, Base::Placement& plc, std::string& jointName);

    bool isJointConnectingPartToGround(App::DocumentObject* joint,
                                       const char* partPropName) override;
    bool isJointTypeConnecting(App::DocumentObject* joint);

    bool isObjInSetOfObjRefs(App::DocumentObject* obj, const std::vector<ObjRef>& pairs);
    void removeUnconnectedJoints(std::vector<App::DocumentObject*>& joints,
                                 std::unordered_set<App::DocumentObject*> groundedObjs);
    void traverseAndMarkConnectedParts(App::DocumentObject* currentPart,
                                       std::vector<ObjRef>& connectedParts,
                                       const std::vector<App::DocumentObject*>& joints);
    std::vector<ObjRef> getConnectedParts(App::DocumentObject* part,
                                          const std::vector<App::DocumentObject*>& joints);
    bool isPartGrounded(App::DocumentObject* part) override;
    bool isPartConnected(App::DocumentObject* part) override;

    std::vector<ObjRef> getDownstreamParts(App::DocumentObject* part,
                                           App::DocumentObject* joint = nullptr) override;
    std::vector<App::DocumentObject*> getUpstreamParts(App::DocumentObject* part, int limit = 0);
    App::DocumentObject* getUpstreamMovingPart(App::DocumentObject* part,
                                               App::DocumentObject*& joint,
                                               std::string& name) override;

    double getObjMass(App::DocumentObject* obj) override;
    void setObjMasses(std::vector<std::pair<App::DocumentObject*, double>> objectMasses) override;

    std::vector<AssemblyLink*> getSubAssemblies();

    std::vector<App::DocumentObject*> getMotionsFromSimulation(App::DocumentObject* sim);

private:
    std::shared_ptr<MbD::ASMTAssembly> mbdAssembly;

    std::unordered_map<App::DocumentObject*, MbDPartData> objectPartMap;
    std::vector<std::pair<App::DocumentObject*, double>> objMasses;
    std::vector<App::DocumentObject*> draggedParts;
    std::vector<App::DocumentObject*> motions;

    std::vector<std::pair<App::DocumentObject*, Base::Placement>> previousPositions;

    bool bundleFixed;
};

}  // namespace Assembly
