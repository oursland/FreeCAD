// SPDX-License-Identifier: LGPL-2.1-or-later
/****************************************************************************
 *                                                                          *
 *   Copyright (c) 2025 Jacob Oursland <jacob.oursland[at]gmail.com>        *
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

#include "Solver.h"

#include <utility>

using namespace Assembly::Solver;

void Item::setName(std::string name)
{
    this->name = std::move(name);
}

std::string Item::getName()
{
    return name;
}

void ItemIJ::setMarkerI(std::shared_ptr<Marker> marker)
{
    this->markerI = std::move(marker);
}

std::shared_ptr<Marker> ItemIJ::getMarkerI()
{
    return markerI;
}

void ItemIJ::setMarkerJ(std::shared_ptr<Marker> marker)
{
    this->markerJ = std::move(marker);
}

std::shared_ptr<Marker> ItemIJ::getMarkerJ()
{
    return markerJ;
}

JointClass Joint::getJointClass()
{
    return jointClass;
}

void Joint::setJointClass(JointClass jointClass)
{
    this->jointClass = jointClass;
}

FixedJoint::FixedJoint()
{
    setJointClass(JointClass::FIXED_JOINT);
}

GearJoint::GearJoint()
{
    setJointClass(JointClass::GEAR_JOINT);
}

BeltJoint::BeltJoint()
{
    setJointClass(JointClass::BELT_JOINT);
}

RevoluteJoint::RevoluteJoint()
{
    setJointClass(JointClass::REVOLUTE_JOINT);
}

CylindricalJoint::CylindricalJoint()
{
    setJointClass(JointClass::CYLINDRICAL_JOINT);
}

TranslationalJoint::TranslationalJoint()
{
    setJointClass(JointClass::TRANSLATIONAL_JOINT);
}

SphericalJoint::SphericalJoint()
{
    setJointClass(JointClass::SPHERICAL_JOINT);
}

ParallelAxesJoint::ParallelAxesJoint()
{
    setJointClass(JointClass::PARALLEL_AXES_JOINT);
}

PerpendicularJoint::PerpendicularJoint()
{
    setJointClass(JointClass::PERPENDICULAR_JOINT);
}

PointInPlaneJoint::PointInPlaneJoint()
{
    setJointClass(JointClass::POINT_IN_PLANE_JOINT);
}

PointInLineJoint::PointInLineJoint()
{
    setJointClass(JointClass::POINT_IN_LINE_JOINT);
}

LineInPlaneJoint::LineInPlaneJoint()
{
    setJointClass(JointClass::LINE_IN_PLANE_JOINT);
}

PlanarJoint::PlanarJoint()
{
    setJointClass(JointClass::PLANAR_JOINT);
}

RevCylJoint::RevCylJoint()
{
    setJointClass(JointClass::REV_CYL_JOINT);
}

CylSphJoint::CylSphJoint()
{
    setJointClass(JointClass::CYL_SPH_JOINT);
}

RackPinionJoint::RackPinionJoint()
{
    setJointClass(JointClass::RACK_PINION_JOINT);
}

ScrewJoint::ScrewJoint()
{
    setJointClass(JointClass::SCREW_JOINT);
}

SphSphJoint::SphSphJoint()
{
    setJointClass(JointClass::SPH_SPH_JOINT);
}

void Limit::setLimitClass(const LimitClass limitClass)
{
    this->limitClass = limitClass;
}

LimitClass Limit::getLimitClass()
{
    return limitClass;
}

RotationLimit::RotationLimit()
{
    setLimitClass(LimitClass::ROTATION_LIMIT);
}

TranslationLimit::TranslationLimit()
{
    setLimitClass(LimitClass::TRANSLATION_LIMIT);
}

void Motion::setMotionClass(const MotionClass motionClass)
{
    this->motionClass = motionClass;
}

MotionClass Motion::getMotionClass()
{
    return motionClass;
}

RotationalMotion::RotationalMotion()
{
    setMotionClass(MotionClass::ROTATIONAL_MOTION);
}

TranslationalMotion::TranslationalMotion()
{
    setMotionClass(MotionClass::TRANSLATIONAL_MOTION);
}

GeneralMotion::GeneralMotion()
{
    setMotionClass(MotionClass::GENERAL_MOTION);
}

void InPlaneJoint::setOffset(const double offset)
{
    this->offset = offset;
};

double InPlaneJoint::getOffset()
{
    return this->offset;
};

void CompoundJoint::setDistance(const double distance)
{
    this->distance = distance;
};

double CompoundJoint::getDistance()
{
    return distance;
};

void AngleJoint::setAngle(const double angle)
{
    this->angle = angle;
}

double AngleJoint::getAngle()
{
    return angle;
}

void GearJoint::setRadiusI(const double radius)
{
    this->radiusI = radius;
}

void GearJoint::setRadiusJ(const double radius)
{
    this->radiusJ = radius;
}

double GearJoint::getRadiusI()
{
    return radiusI;
}

double GearJoint::getRadiusJ()
{
    return radiusJ;
}

void BeltJoint::setRadiusI(const double radius)
{
    this->radiusI = radius;
}

void BeltJoint::setRadiusJ(const double radius)
{
    this->radiusJ = radius;
}

double BeltJoint::getRadiusI()
{
    return radiusI;
}

double BeltJoint::getRadiusJ()
{
    return radiusJ;
}

void RackPinionJoint::setPitchRadius(const double pitchRadius)
{
    this->pitchRadius = pitchRadius;
}

double RackPinionJoint::getPitchRadius()
{
    return pitchRadius;
}

void ScrewJoint::setPitch(const double pitch)
{
    this->pitch = pitch;
}

double ScrewJoint::getPitch()
{
    return pitch;
}

void Limit::setLimit(const double limit)
{
    this->limit = limit;
};

double Limit::getLimit()
{
    return limit;
};

void Limit::setTolerance(const double tolerance)
{
    this->tolerance = tolerance;
};

double Limit::getTolerance()
{
    return tolerance;
};

void Limit::setType(const LimitType type)
{
    this->type = type;
};

LimitType Limit::getType()
{
    return type;
};

Base::Vector3d SpatialItem::getPosition3D()
{
    return placement.getPosition();
}

Base::Rotation SpatialItem::getRotation()
{
    return placement.getRotation();
}

Base::Placement SpatialItem::getPlacement()
{
    return placement;
}

void SpatialItem::setPosition3D(const Base::Vector3d position)
{
    this->placement.setPosition(position);
}

void SpatialItem::setRotation(const Base::Rotation rotation)
{
    this->placement.setRotation(rotation);
}

void SpatialItem::setPlacement(const Base::Placement placement)
{
    this->placement = placement;
}

void SpatialContainer::setPrincipalMassMarker(std::shared_ptr<PrincipalMassMarker> marker)
{
    this->principalMassMarker = std::move(marker);
}

std::shared_ptr<PrincipalMassMarker> SpatialContainer::getPrincipalMassMarker()
{
    return principalMassMarker;
}

void SpatialContainer::addMarker(std::shared_ptr<Marker> marker)
{
    markers.push_back(marker);
}

std::vector<std::shared_ptr<Marker>> SpatialContainer::getMarkers()
{
    return markers;
}

void PrincipalMassMarker::setMass(const double mass)
{
    this->mass = mass;
}

void PrincipalMassMarker::setDensity(const double density)
{
    this->density = density;
}

void PrincipalMassMarker::setMomentOfInertias(const std::array<double, 3> inertias)
{
    this->inertias = inertias;
}

double PrincipalMassMarker::getMass()
{
    return mass;
}

double PrincipalMassMarker::getDensity()
{
    return density;
}

std::array<double, 3> PrincipalMassMarker::getMomentOfInertias()
{
    return inertias;
}

void SimulationParameters::setTimeStart(double timeStart)
{
    this->timeStart = timeStart;
};

double SimulationParameters::getTimeStart()
{
    return timeStart;
};

void SimulationParameters::setTimeEnd(double timeEnd)
{
    this->timeEnd = timeEnd;
};

double SimulationParameters::getTimeEnd()
{
    return timeEnd;
};

void SimulationParameters::setTimeStepOutput(double timeStepOutput)
{
    this->timeStepOutput = timeStepOutput;
}

double SimulationParameters::getTimeStepOutput()
{
    return timeStepOutput;
}

std::shared_ptr<Part> AssemblySolver::makePart()
{
    return std::make_shared<Part>();
}

std::shared_ptr<Marker> AssemblySolver::makeMarker()
{
    return std::make_shared<Marker>();
}

std::shared_ptr<PrincipalMassMarker> AssemblySolver::makePrincipalMassMarker()
{
    return std::make_shared<PrincipalMassMarker>();
}

std::shared_ptr<FixedJoint> AssemblySolver::makeFixedJoint()
{
    return std::make_shared<FixedJoint>();
}

std::shared_ptr<RevoluteJoint> AssemblySolver::makeRevoluteJoint()
{
    return std::make_shared<RevoluteJoint>();
}

std::shared_ptr<CylindricalJoint> AssemblySolver::makeCylindricalJoint()
{
    return std::make_shared<CylindricalJoint>();
}

std::shared_ptr<TranslationalJoint> AssemblySolver::makeTranslationalJoint()
{
    return std::make_shared<TranslationalJoint>();
}

std::shared_ptr<SphericalJoint> AssemblySolver::makeSphericalJoint()
{
    return std::make_shared<SphericalJoint>();
}

std::shared_ptr<ParallelAxesJoint> AssemblySolver::makeParallelJoint()
{
    return std::make_shared<ParallelAxesJoint>();
}

std::shared_ptr<PerpendicularJoint> AssemblySolver::makePerpendicularJoint()
{
    return std::make_shared<PerpendicularJoint>();
}

std::shared_ptr<AngleJoint> AssemblySolver::makeAngleJoint()
{
    return std::make_shared<AngleJoint>();
}

std::shared_ptr<RackPinionJoint> AssemblySolver::makeRackPinionJoint()
{
    return std::make_shared<RackPinionJoint>();
}

std::shared_ptr<ScrewJoint> AssemblySolver::makeScrewJoint()
{
    return std::make_shared<ScrewJoint>();
}

std::shared_ptr<GearJoint> AssemblySolver::makeGearJoint()
{
    return std::make_shared<GearJoint>();
}

std::shared_ptr<BeltJoint> AssemblySolver::makeBeltJoint()
{
    return std::make_shared<BeltJoint>();
}

std::shared_ptr<SphSphJoint> AssemblySolver::makeCylSphJoint()
{
    return std::make_shared<SphSphJoint>();
}

std::shared_ptr<SphSphJoint> AssemblySolver::makeSphSphJoint()
{
    return std::make_shared<SphSphJoint>();
}

std::shared_ptr<RevCylJoint> AssemblySolver::makeRevCylJoint()
{
    return std::make_shared<RevCylJoint>();
}

std::shared_ptr<PlanarJoint> AssemblySolver::makePlanarJoint()
{
    return std::make_shared<PlanarJoint>();
}

std::shared_ptr<PointInPlaneJoint> AssemblySolver::makePointInPlaneJoint()
{
    return std::make_shared<PointInPlaneJoint>();
}

std::shared_ptr<LineInPlaneJoint> AssemblySolver::makeLineInPlaneJoint()
{
    return std::make_shared<LineInPlaneJoint>();
}

std::shared_ptr<Joint> AssemblySolver::makeDistanceJoint()
{
    return std::make_shared<Joint>();
}

std::shared_ptr<RotationLimit> AssemblySolver::makeRotationLimit()
{
    return std::make_shared<RotationLimit>();
}

std::shared_ptr<TranslationLimit> AssemblySolver::makeTranslationLimit()
{
    return std::make_shared<TranslationLimit>();
}

std::shared_ptr<RotationalMotion> AssemblySolver::makeRotationalMotion()
{
    return std::make_shared<RotationalMotion>();
}

std::shared_ptr<TranslationalMotion> AssemblySolver::makeTranslationalMotion()
{
    return std::make_shared<TranslationalMotion>();
}

std::shared_ptr<GeneralMotion> AssemblySolver::makeGeneralMotion()
{
    return std::make_shared<GeneralMotion>();
}
