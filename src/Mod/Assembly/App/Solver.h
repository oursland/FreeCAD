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

#pragma once

#include "Base/Placement.h"
#include "Base/Rotation.h"
#include "Base/Vector3D.h"
#include "Mod/Assembly/App/AssemblyUtils.h"

#include <array>
#include <memory>

// Notes:
//   * The document object is the source of truth for the actual state of the system.  Consequently,
//     calls to AssemblyObject::solve() starts by clearing the existing assembly and adding the
//     components to be solved.  This is a somewhat inefficient process, but it ensures that there
//     is not a mismatch between the document model and what was previously inserted into the
//     solver.
//   * In principle, the various classes for parts, joints, and limits are intended to be a 1:1
//     mapping with the associated document object.  The document model and the OndselSolver were
//     developed jointly, which means the abstract solver interface will closely match that of the
//     OndselSolve, but not likely any other solver.
//   * Access to these parts, joints, and limits classes' members shall be through member functions
//     to facilitate specialization through overriding, if necessary for a given solver
//     implementation.
//   * The assembly's `addPart()`, `addJoint()`, and `addLimit()` functions should translate the
//     Assembly::Solver class to the implementation-specific version and add it to the solver's
//     internal representation.
//   * A bare minimum implementation will implement the `Solver::Assembly` interface and
//   `Solver::AssemblySolver::makeAssembly`

namespace Assembly::Solver
{

class Marker;

class Item
{
public:
    virtual ~Item() = default;

    virtual void setName(std::string name);
    virtual std::string getName();

private:
    std::string name;
};

class ItemIJ: public Item
{
public:
    virtual ~ItemIJ() = default;

    virtual void setMarkerI(std::shared_ptr<Marker> marker);
    virtual void setMarkerJ(std::shared_ptr<Marker> marker);
    virtual std::shared_ptr<Marker> getMarkerI();
    virtual std::shared_ptr<Marker> getMarkerJ();

private:
    std::shared_ptr<Marker> markerI;
    std::shared_ptr<Marker> markerJ;
};

class ConstraintSet: public ItemIJ
{
public:
    virtual ~ConstraintSet() = default;
};

enum class JointClass
{
    JOINT,
    FIXED_JOINT,
    REVOLUTE_JOINT,
    CYLINDRICAL_JOINT,
    TRANSLATIONAL_JOINT,
    SPHERICAL_JOINT,
    PARALLEL_AXES_JOINT,
    PERPENDICULAR_JOINT,
    ANGLE_JOINT,
    RACK_PINION_JOINT,
    SCREW_JOINT,
    GEAR_JOINT,
    BELT_JOINT,
    SPH_SPH_JOINT,
    REV_CYL_JOINT,
    PLANAR_JOINT,
    CYL_SPH_JOINT,
    POINT_IN_PLANE_JOINT,
    POINT_IN_LINE_JOINT,
    LINE_IN_PLANE_JOINT,
};

class Joint: public ConstraintSet
{
public:
    virtual ~Joint() = default;
    JointClass getJointClass();

protected:
    void setJointClass(JointClass jointClass);

private:
    JointClass jointClass = JointClass::JOINT;
};

class AtPointJoint: public Joint
{
public:
    virtual ~AtPointJoint() = default;
};

class InLineJoint: public Joint
{
public:
    virtual ~InLineJoint() = default;
};

class InPlaneJoint: public Joint
{
public:
    virtual ~InPlaneJoint() = default;

    virtual void setOffset(double offset);
    virtual double getOffset();

private:
    double offset = 0.0;
};

class CompoundJoint: public Joint
{
public:
    virtual ~CompoundJoint() = default;

    virtual void setDistance(double distance);
    virtual double getDistance();

private:
    double distance = 0.0;
};

class AngleJoint: public Joint
{
public:
    virtual ~AngleJoint() = default;

    virtual void setAngle(double angle);
    virtual double getAngle();

private:
    double angle = 0.0;
};

class FixedJoint: public AtPointJoint
{
public:
    FixedJoint();
    virtual ~FixedJoint() = default;
};

class GearJoint: public Joint
{
public:
    GearJoint();
    virtual ~GearJoint() = default;

    virtual void setRadiusI(double radius);
    virtual void setRadiusJ(double radius);
    virtual double getRadiusI();
    virtual double getRadiusJ();

private:
    double radiusI = 1.0;
    double radiusJ = 1.0;
};

class BeltJoint: public Joint
{
public:
    BeltJoint();
    virtual ~BeltJoint() = default;

    virtual void setRadiusI(double radius);
    virtual void setRadiusJ(double radius);
    virtual double getRadiusI();
    virtual double getRadiusJ();

private:
    double radiusI = 1.0;
    double radiusJ = 1.0;
};

class RevoluteJoint: public AtPointJoint
{
public:
    RevoluteJoint();
    virtual ~RevoluteJoint() = default;
};

class CylindricalJoint: public InLineJoint
{
public:
    CylindricalJoint();
    virtual ~CylindricalJoint() = default;
};

class TranslationalJoint: public InLineJoint
{
public:
    TranslationalJoint();
    virtual ~TranslationalJoint() = default;
};

class SphericalJoint: public AtPointJoint
{
public:
    SphericalJoint();
    virtual ~SphericalJoint() = default;
};

class ParallelAxesJoint: public Joint
{
public:
    ParallelAxesJoint();
    virtual ~ParallelAxesJoint() = default;
};

class PerpendicularJoint: public Joint
{
public:
    PerpendicularJoint();
    virtual ~PerpendicularJoint() = default;
};

class PointInPlaneJoint: public InPlaneJoint
{
public:
    PointInPlaneJoint();
    virtual ~PointInPlaneJoint() = default;
};

class PointInLineJoint: public InLineJoint
{
public:
    PointInLineJoint();
    virtual ~PointInLineJoint() = default;
};

class LineInPlaneJoint: public InPlaneJoint
{
public:
    LineInPlaneJoint();
    virtual ~LineInPlaneJoint() = default;
};

class PlanarJoint: public InPlaneJoint
{
public:
    PlanarJoint();
    virtual ~PlanarJoint() = default;
};

class RevCylJoint: public CompoundJoint
{
public:
    RevCylJoint();
    virtual ~RevCylJoint() = default;
};

class CylSphJoint: public CompoundJoint
{
public:
    CylSphJoint();
    virtual ~CylSphJoint() = default;
};

class RackPinionJoint: public Joint
{
public:
    RackPinionJoint();
    virtual ~RackPinionJoint() = default;

    virtual void setPitchRadius(double pitchRadius);
    virtual double getPitchRadius();

private:
    double pitchRadius = 1.0;
};

class ScrewJoint: public Joint
{
public:
    ScrewJoint();
    virtual ~ScrewJoint() = default;

    virtual void setPitch(double pitch);
    virtual double getPitch();

private:
    double pitch = 1.0;
};

class SphSphJoint: public CompoundJoint
{
public:
    SphSphJoint();
    virtual ~SphSphJoint() = default;
};

enum class LimitClass
{
    LIMIT,
    ROTATION_LIMIT,
    TRANSLATION_LIMIT,
};

enum class LimitType
{
    NO_LIMIT,
    LESS_THAN_OR_EQUAL,
    GREATER_THAN_OR_EQUAL,
};

class Limit: public ConstraintSet
{
public:
    virtual ~Limit() = default;

    virtual void setLimit(double limit);
    virtual double getLimit();

    virtual void setTolerance(double tolerance);
    virtual double getTolerance();

    virtual void setType(LimitType type);
    virtual LimitType getType();

    virtual LimitClass getLimitClass();

protected:
    virtual void setLimitClass(LimitClass limitClass);

private:
    LimitClass limitClass = LimitClass::LIMIT;
    LimitType type = LimitType::NO_LIMIT;
    double limit = 0.0;
    double tolerance = 1e-9;
};

class RotationLimit: public Limit
{
public:
    RotationLimit();
    virtual ~RotationLimit() = default;
};

class TranslationLimit: public Limit
{
public:
    TranslationLimit();
    virtual ~TranslationLimit() = default;
};

enum class MotionClass
{
    MOTION,
    ROTATIONAL_MOTION,
    TRANSLATIONAL_MOTION,
    GENERAL_MOTION,
};

class Motion: public ConstraintSet
{
public:
    virtual ~Motion() = default;

    MotionClass getMotionClass();

protected:
    void setMotionClass(MotionClass motionClass);

private:
    MotionClass motionClass = MotionClass::MOTION;
};

class RotationalMotion: public Motion
{
public:
    RotationalMotion();
    virtual ~RotationalMotion() = default;
};

class TranslationalMotion: public Motion
{
public:
    TranslationalMotion();
    virtual ~TranslationalMotion() = default;
};

class GeneralMotion: public Motion
{
public:
    GeneralMotion();
    virtual ~GeneralMotion() = default;
};

class SpatialItem: public Item
{
public:
    virtual ~SpatialItem() = default;

    virtual Base::Vector3d getPosition3D();
    virtual Base::Rotation getRotation();
    virtual Base::Placement getPlacement();
    virtual void setPosition3D(Base::Vector3d position);
    virtual void setRotation(Base::Rotation rotation);
    virtual void setPlacement(Base::Placement placement);

private:
    Base::Placement placement;
};

class Marker: public SpatialItem
{
public:
    virtual ~Marker() = default;
};

class PrincipalMassMarker: public Marker
{
public:
    virtual ~PrincipalMassMarker() = default;

    virtual void setMass(double mass);
    virtual void setDensity(double density);
    virtual void setMomentOfInertias(std::array<double, 3> inertias);
    virtual double getMass();
    virtual double getDensity();
    virtual std::array<double, 3> getMomentOfInertias();

private:
    double mass = 1.0;
    double density = 1.0;
    std::array<double, 3> inertias;
};

class SpatialContainer: public SpatialItem
{
public:
    virtual ~SpatialContainer() = default;

    virtual void addMarker(std::shared_ptr<Marker> marker);
    virtual std::vector<std::shared_ptr<Marker>> getMarkers();

    virtual void setPrincipalMassMarker(std::shared_ptr<PrincipalMassMarker> marker);
    virtual std::shared_ptr<PrincipalMassMarker> getPrincipalMassMarker();

private:
    std::shared_ptr<PrincipalMassMarker> principalMassMarker;
    std::vector<std::shared_ptr<Marker>> markers;
};

class Part: public SpatialContainer
{
public:
    virtual ~Part() = default;

    // update placement?
};

class SimulationParameters: public Item
{
public:
    virtual ~SimulationParameters() = default;

    virtual void setTimeStart(double timeStart);
    virtual double getTimeStart();
    virtual void setTimeEnd(double timeEnd);
    virtual double getTimeEnd();
    virtual void setTimeStepOutput(double timeStepOutput);
    virtual double getTimeStepOutput();

private:
    double timeStart = 0.0;
    double timeEnd = 1.0;
    double timeStepOutput = 1.0 / 30.0;
};

class Assembly: public SpatialContainer
{
public:
    virtual ~Assembly() = default;

    virtual void addPart(std::shared_ptr<Part> part) = 0;
    void addMarker(std::shared_ptr<Marker> marker) override = 0;
    virtual void addJoint(std::shared_ptr<Joint> joint) = 0;
    virtual void addLimit(std::shared_ptr<Limit> limit) = 0;
    virtual void addMotion(std::shared_ptr<Motion> motion) = 0;

    virtual void solve() = 0;

    // UI drag interaction
    virtual void preDrag() = 0;
    virtual void dragStep(std::shared_ptr<std::vector<std::shared_ptr<Part>>> parts) = 0;
    virtual void postDrag() = 0;

    // simulation
    virtual void
    setSimulationParameters(std::shared_ptr<SimulationParameters> simulationParameters) = 0;
    virtual std::shared_ptr<SimulationParameters> getSimulationParameters() = 0;
    virtual size_t numberOfFrames() = 0;
    virtual void updateForFrame(size_t index) = 0;

    virtual void exportFile(std::string filename) = 0;
};

class AssemblySolver
{
public:
    virtual ~AssemblySolver() = default;

    virtual std::shared_ptr<Assembly> makeAssembly() = 0;

    virtual std::shared_ptr<Part> makePart();

    virtual std::shared_ptr<Marker> makeMarker();
    virtual std::shared_ptr<PrincipalMassMarker> makePrincipalMassMarker();

    virtual std::shared_ptr<FixedJoint> makeFixedJoint();
    virtual std::shared_ptr<RevoluteJoint> makeRevoluteJoint();
    virtual std::shared_ptr<CylindricalJoint> makeCylindricalJoint();
    virtual std::shared_ptr<TranslationalJoint> makeTranslationalJoint();
    virtual std::shared_ptr<SphericalJoint> makeSphericalJoint();
    virtual std::shared_ptr<ParallelAxesJoint> makeParallelJoint();
    virtual std::shared_ptr<PerpendicularJoint> makePerpendicularJoint();
    virtual std::shared_ptr<AngleJoint> makeAngleJoint();
    virtual std::shared_ptr<RackPinionJoint> makeRackPinionJoint();
    virtual std::shared_ptr<ScrewJoint> makeScrewJoint();
    virtual std::shared_ptr<GearJoint> makeGearJoint();
    virtual std::shared_ptr<BeltJoint> makeBeltJoint();
    virtual std::shared_ptr<SphSphJoint> makeCylSphJoint();
    virtual std::shared_ptr<SphSphJoint> makeSphSphJoint();
    virtual std::shared_ptr<RevCylJoint> makeRevCylJoint();
    virtual std::shared_ptr<PlanarJoint> makePlanarJoint();
    virtual std::shared_ptr<PointInPlaneJoint> makePointInPlaneJoint();
    virtual std::shared_ptr<LineInPlaneJoint> makeLineInPlaneJoint();

    // distance joint??
    virtual std::shared_ptr<Joint> makeDistanceJoint();

    virtual std::shared_ptr<RotationLimit> makeRotationLimit();
    virtual std::shared_ptr<TranslationLimit> makeTranslationLimit();

    virtual std::shared_ptr<RotationalMotion> makeRotationalMotion();
    virtual std::shared_ptr<TranslationalMotion> makeTranslationalMotion();
    virtual std::shared_ptr<GeneralMotion> makeGeneralMotion();
};

}  // namespace Assembly::Solver
