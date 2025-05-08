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

#include "OndselSolver.h"

#include <OndselSolver/CREATE.h>
#include <OndselSolver/ASMTSimulationParameters.h>
#include <OndselSolver/ASMTAssembly.h>
#include <OndselSolver/ASMTMarker.h>
#include <OndselSolver/ASMTPart.h>
#include <OndselSolver/ASMTJoint.h>
#include <OndselSolver/ASMTAngleJoint.h>
#include <OndselSolver/ASMTFixedJoint.h>
#include <OndselSolver/ASMTGearJoint.h>
#include <OndselSolver/ASMTRevoluteJoint.h>
#include <OndselSolver/ASMTCylindricalJoint.h>
#include <OndselSolver/ASMTTranslationalJoint.h>
#include <OndselSolver/ASMTSphericalJoint.h>
#include <OndselSolver/ASMTParallelAxesJoint.h>
#include <OndselSolver/ASMTPerpendicularJoint.h>
#include <OndselSolver/ASMTPointInPlaneJoint.h>
#include <OndselSolver/ASMTPointInLineJoint.h>
#include <OndselSolver/ASMTLineInPlaneJoint.h>
#include <OndselSolver/ASMTPlanarJoint.h>
#include <OndselSolver/ASMTRevCylJoint.h>
#include <OndselSolver/ASMTCylSphJoint.h>
#include <OndselSolver/ASMTRackPinionJoint.h>
#include <OndselSolver/ASMTRotationLimit.h>
#include <OndselSolver/ASMTTranslationLimit.h>
#include <OndselSolver/ASMTRotationalMotion.h>
#include <OndselSolver/ASMTTranslationalMotion.h>
#include <OndselSolver/ASMTGeneralMotion.h>
#include <OndselSolver/ASMTScrewJoint.h>
#include <OndselSolver/ASMTSphSphJoint.h>
#include <OndselSolver/ASMTTime.h>
#include <OndselSolver/ASMTConstantGravity.h>
#include <OndselSolver/ExternalSystem.h>
#include <OndselSolver/enum.h>
#include <memory>
#include <string>

#include "AssemblyObject.h"
#include "Base/Console.h"
#include "Base/Placement.h"
#include "Mod/Assembly/App/Solver.h"

FC_LOG_LEVEL_INIT("OndselSolver", true, true, true)

using namespace Assembly;
using namespace Assembly::Solver;

OndselPart::OndselPart()
{
    asmtPart = MbD::CREATE<MbD::ASMTPart>::With();
}

void OndselPart::addMarker(std::shared_ptr<Marker> marker)
{
    auto asmtMarker = MbD::CREATE<MbD::ASMTMarker>::With();
    asmtMarker->setName(marker->getName());

    Base::Vector3d pos = marker->getPosition3D();
    asmtMarker->setPosition3D(pos.x, pos.y, pos.z);
    Base::Rotation rot = marker->getRotation();
    Base::Matrix4D mat;
    rot.getValue(mat);
    Base::Vector3d r0 = mat.getRow(0);
    Base::Vector3d r1 = mat.getRow(1);
    Base::Vector3d r2 = mat.getRow(2);
    asmtMarker->setRotationMatrix(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z);

    asmtPart->addMarker(asmtMarker);
}

std::shared_ptr<MbD::ASMTPart> OndselPart::getAsmtPart()
{
    // set name
    asmtPart->setName(getName());

    // set mass marker
    auto asmtMassMarker = MbD::CREATE<MbD::ASMTPrincipalMassMarker>::With();
    // TODO: ???
    // asmtMassMarker->setMass(part->getMass());
    // asmtMassMarker->setDensity(part->getDensity());
    // asmtMassMarker->setMomentOfInertias(part->getMomentOfInertias());
    asmtPart->setPrincipalMassMarker(asmtMassMarker);

    // set placement
    auto placement = getPlacement();
    Base::Vector3d pos = placement.getPosition();
    asmtPart->setPosition3D(pos.x, pos.y, pos.z);
    Base::Rotation rot = placement.getRotation();
    Base::Matrix4D mat;
    rot.getValue(mat);
    Base::Vector3d r0 = mat.getRow(0);
    Base::Vector3d r1 = mat.getRow(1);
    Base::Vector3d r2 = mat.getRow(2);
    asmtPart->setRotationMatrix(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z);

    return asmtPart;
}

void OndselPart::updatePart()
{
    // set name
    setName(asmtPart->name);

    // set placement
    double x, y, z;
    asmtPart->getPosition3D(x, y, z);
    Base::Vector3d position {x, y, z};
    setPosition3D(position);

    double q0, q1, q2, q3;
    asmtPart->getQuarternions(q0, q1, q2, q3);
    Base::Rotation rotation {q0, q1, q2, q3};
    setRotation(rotation);
}

OndselAssembly::OndselAssembly(AssemblyObject* assemblyObject)
{
    asmtAssembly = MbD::CREATE<MbD::ASMTAssembly>::With();

    setExternalSystem(assemblyObject);

    ParameterGrp::handle hPgr = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Assembly");

    asmtAssembly->setDebug(hPgr->GetBool("LogSolverDebug", false));
}

void OndselAssembly::addMarker(std::shared_ptr<Marker> marker)
{
    auto asmtMarker = MbD::CREATE<MbD::ASMTMarker>::With();
    asmtMarker->setName(marker->getName());

    Base::Vector3d pos = marker->getPosition3D();
    asmtMarker->setPosition3D(pos.x, pos.y, pos.z);
    Base::Rotation rot = marker->getRotation();
    Base::Matrix4D mat;
    rot.getValue(mat);
    Base::Vector3d r0 = mat.getRow(0);
    Base::Vector3d r1 = mat.getRow(1);
    Base::Vector3d r2 = mat.getRow(2);
    asmtMarker->setRotationMatrix(r0.x, r0.y, r0.z, r1.x, r1.y, r1.z, r2.x, r2.y, r2.z);

    asmtAssembly->addMarker(asmtMarker);
}

void OndselAssembly::addJoint(std::shared_ptr<Joint> joint)
{
    switch (joint->getJointClass()) {
        case JointClass::FIXED_JOINT: {
            auto solverJoint = std::static_pointer_cast<FixedJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTFixedJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::REVOLUTE_JOINT: {
            auto solverJoint = std::static_pointer_cast<RevoluteJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTRevoluteJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::CYLINDRICAL_JOINT: {
            auto solverJoint = std::static_pointer_cast<CylindricalJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTCylindricalJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::TRANSLATIONAL_JOINT: {
            auto solverJoint = std::static_pointer_cast<TranslationalJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTTranslationalJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::SPHERICAL_JOINT: {
            auto solverJoint = std::static_pointer_cast<SphericalJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTSphericalJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::PARALLEL_AXES_JOINT: {
            auto solverJoint = std::static_pointer_cast<ParallelAxesJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTParallelAxesJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::PERPENDICULAR_JOINT: {
            auto solverJoint = std::static_pointer_cast<PerpendicularJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTPerpendicularJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::ANGLE_JOINT: {
            auto solverJoint = std::static_pointer_cast<AngleJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTAngleJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->theIzJz = solverJoint->getAngle();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::RACK_PINION_JOINT: {
            auto solverJoint = std::static_pointer_cast<RackPinionJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTRackPinionJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->pitchRadius = solverJoint->getPitchRadius();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::SCREW_JOINT: {
            auto solverJoint = std::static_pointer_cast<ScrewJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTScrewJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->pitch = solverJoint->getPitch();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::GEAR_JOINT: {
            auto solverJoint = std::static_pointer_cast<GearJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTGearJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->radiusI = solverJoint->getRadiusI();
            asmtJoint->radiusJ = solverJoint->getRadiusJ();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::BELT_JOINT: {
            auto solverJoint = std::static_pointer_cast<BeltJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTGearJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->radiusI = solverJoint->getRadiusI();
            asmtJoint->radiusJ = solverJoint->getRadiusJ();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::SPH_SPH_JOINT: {
            auto solverJoint = std::static_pointer_cast<SphSphJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTSphSphJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->distanceIJ = solverJoint->getDistance();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::REV_CYL_JOINT: {
            auto solverJoint = std::static_pointer_cast<RevCylJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTRevCylJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->distanceIJ = solverJoint->getDistance();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::PLANAR_JOINT: {
            auto solverJoint = std::static_pointer_cast<PlanarJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTPlanarJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->offset = solverJoint->getOffset();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::CYL_SPH_JOINT: {
            auto solverJoint = std::static_pointer_cast<CylSphJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTCylSphJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->distanceIJ = solverJoint->getDistance();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::POINT_IN_PLANE_JOINT: {
            auto solverJoint = std::static_pointer_cast<PointInPlaneJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTPointInPlaneJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->offset = solverJoint->getOffset();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::POINT_IN_LINE_JOINT: {
            auto solverJoint = std::static_pointer_cast<PointInLineJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTPointInLineJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::LINE_IN_PLANE_JOINT: {
            auto solverJoint = std::static_pointer_cast<LineInPlaneJoint>(joint);
            auto asmtJoint = MbD::CREATE<MbD::ASMTLineInPlaneJoint>::With();
            asmtJoint->setName(solverJoint->getName());
            asmtJoint->setMarkerI(solverJoint->getMarkerI()->getName());
            asmtJoint->setMarkerJ(solverJoint->getMarkerJ()->getName());
            asmtJoint->offset = solverJoint->getOffset();
            asmtAssembly->addJoint(asmtJoint);
            break;
        }

        case JointClass::JOINT:
        default:
            FC_WARN("Unknown joint type!");
    }
}

void OndselAssembly::addLimit(std::shared_ptr<Limit> limit)
{
    switch (limit->getLimitClass()) {
        case LimitClass::ROTATION_LIMIT: {
            auto solverLimit = std::static_pointer_cast<RotationLimit>(limit);
            auto asmtLimit = MbD::CREATE<MbD::ASMTRotationLimit>::With();
            asmtLimit->setName(solverLimit->getName());
            asmtLimit->setMarkerI(solverLimit->getMarkerI()->getName());
            asmtLimit->setMarkerJ(solverLimit->getMarkerJ()->getName());
            switch (solverLimit->getType()) {
                case LimitType::LESS_THAN_OR_EQUAL:
                    asmtLimit->settype("=<");
                    break;

                case LimitType::GREATER_THAN_OR_EQUAL:
                    asmtLimit->settype("=>");
                    break;

                case LimitType::NO_LIMIT:
                default:
                    FC_WARN("Unknown limit type!");
            }
            asmtLimit->setlimit(std::to_string(solverLimit->getLimit()));
            asmtLimit->settol(std::to_string(solverLimit->getTolerance()));
            asmtAssembly->addLimit(asmtLimit);
            break;
        }

        case LimitClass::TRANSLATION_LIMIT: {
            auto solverLimit = std::static_pointer_cast<TranslationLimit>(limit);
            auto asmtLimit = MbD::CREATE<MbD::ASMTTranslationLimit>::With();
            asmtLimit->setName(solverLimit->getName());
            asmtLimit->setMarkerI(solverLimit->getMarkerI()->getName());
            asmtLimit->setMarkerJ(solverLimit->getMarkerJ()->getName());
            switch (solverLimit->getType()) {
                case LimitType::LESS_THAN_OR_EQUAL:
                    asmtLimit->settype("=<");
                    break;

                case LimitType::GREATER_THAN_OR_EQUAL:
                    asmtLimit->settype("=>");
                    break;

                case LimitType::NO_LIMIT:
                default:
                    FC_WARN("Unknown limit type!");
            }
            asmtLimit->setlimit(std::to_string(solverLimit->getLimit()));
            asmtLimit->settol(std::to_string(solverLimit->getTolerance()));
            asmtAssembly->addLimit(asmtLimit);
            break;
        }

        case LimitClass::LIMIT:
        default:
            FC_WARN("Unknown limit type!");
    }
}

void OndselAssembly::addMotion(std::shared_ptr<Motion> motion)
{
    switch (motion->getMotionClass()) {
        case MotionClass::ROTATIONAL_MOTION: {
            auto solverMotion = std::static_pointer_cast<RotationalMotion>(motion);
            auto asmtMotion = MbD::CREATE<MbD::ASMTRotationalMotion>::With();
            asmtMotion->setName(solverMotion->getName());
            asmtMotion->setMarkerI(solverMotion->getMarkerI()->getName());
            asmtMotion->setMarkerJ(solverMotion->getMarkerJ()->getName());
            // TODO
            // asmtMotion->setRotationZ(solverMotion->getRotation());
            break;
        }

        case MotionClass::TRANSLATIONAL_MOTION: {
            auto solverMotion = std::static_pointer_cast<TranslationalMotion>(motion);
            auto asmtMotion = MbD::CREATE<MbD::ASMTTranslationalMotion>::With();
            asmtMotion->setName(solverMotion->getName());
            asmtMotion->setMarkerI(solverMotion->getMarkerI()->getName());
            asmtMotion->setMarkerJ(solverMotion->getMarkerJ()->getName());
            // TODO
            // asmtMotion->setTranslationZ(solverMotion->getTranslation());
            break;
        }

        case MotionClass::GENERAL_MOTION: {
            auto solverMotion = std::static_pointer_cast<GeneralMotion>(motion);
            auto asmtMotion = MbD::CREATE<MbD::ASMTGeneralMotion>::With();
            asmtMotion->setName(solverMotion->getName());
            asmtMotion->setMarkerI(solverMotion->getMarkerI()->getName());
            asmtMotion->setMarkerJ(solverMotion->getMarkerJ()->getName());
            // TODO
            // asmtMotion->rIJI->atiput(2, motionType == "Angular" ? formula2 : formula);
            // asmtMotion->angIJJ->atiput(2, motionType == "Angular" ? formula : formula2);
            break;
        }

        case MotionClass::MOTION:
        default:
            FC_WARN("Unknown motion type!");
    }
}

void OndselAssembly::addPart(std::shared_ptr<Part> part)
{
    auto ondselPart = std::static_pointer_cast<OndselPart>(part);
    auto asmtPart = ondselPart->getAsmtPart();
    asmtAssembly->addPart(asmtPart);
}

void OndselAssembly::solve()
{
    asmtAssembly->runKINEMATIC();
}

void OndselAssembly::preDrag()
{
    asmtAssembly->runPreDrag();
}

void OndselAssembly::dragStep(std::shared_ptr<std::vector<std::shared_ptr<Part>>> parts)
{
    // extract vector of ASMTParts
    std::shared_ptr<std::vector<std::shared_ptr<MbD::ASMTPart>>> asmtParts;
    for (const auto& part : *parts) {
        auto ondselPart = std::static_pointer_cast<OndselPart>(part);
        auto asmtPart = ondselPart->getAsmtPart();
        asmtParts->push_back(asmtPart);
    }

    // run the drag step, which updates the placements of the ASMTParts
    asmtAssembly->runDragStep(asmtParts);

    // update placements from the AMSTParts
    for (const auto& part : *parts) {
        auto ondselPart = std::static_pointer_cast<OndselPart>(part);
        ondselPart->updatePart();
    }
}

void OndselAssembly::postDrag()
{
    asmtAssembly->runPostDrag();
}

void OndselAssembly::setSimulationParameters(
    std::shared_ptr<SimulationParameters> simulationParameters)
{
    this->simulationParameters = simulationParameters;
}

std::shared_ptr<SimulationParameters> OndselAssembly::getSimulationParameters()
{}

size_t OndselAssembly::numberOfFrames()
{}

void OndselAssembly::updateForFrame(size_t index)
{}

void OndselAssembly::exportFile(std::string filename)
{
    asmtAssembly->outputFile(filename);
}

void OndselAssembly::setExternalSystem(AssemblyObject* assemblyObject)
{
    asmtAssembly->externalSystem->freecadAssemblyObject = assemblyObject;
}

std::shared_ptr<Part> OndselSolver::makePart()
{
    auto ondselPart = std::make_shared<OndselPart>();
    return ondselPart;
}

OndselSolver::OndselSolver(AssemblyObject* assemblyObject)
    : assemblyObject(assemblyObject)
{}

std::shared_ptr<Solver::Assembly> OndselSolver::makeAssembly()
{
    auto assembly = std::make_shared<OndselAssembly>(assemblyObject);
    assembly->setName("OndselAssembly");

    return assembly;
}
