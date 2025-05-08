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

#include "OndselSolver/ASMTPart.h"
#include "Solver.h"

#include "Mod/Assembly/App/AssemblyObject.h"
#include <memory>

namespace MbD
{
class ASMTPart;
class ASMTAssembly;
class ASMTJoint;
class ASMTMarker;
class ASMTPart;
}  // namespace MbD

namespace Assembly::Solver
{

// OndselPart is defined as the solver updates the parts during solve
class OndselPart: public Part
{
public:
    OndselPart();
    virtual ~OndselPart() = default;

    void addMarker(std::shared_ptr<Marker> marker) override;

    std::shared_ptr<MbD::ASMTPart> getAsmtPart();
    void updatePart();

private:
    std::shared_ptr<MbD::ASMTPart> asmtPart;
};

class OndselAssembly: public Assembly
{
public:
    explicit OndselAssembly(AssemblyObject* assemblyObject);
    virtual ~OndselAssembly() = default;

    void addPart(std::shared_ptr<Part> part) override;
    void addMarker(std::shared_ptr<Marker> marker) override;
    void addJoint(std::shared_ptr<Joint> joint) override;
    void addLimit(std::shared_ptr<Limit> limit) override;
    void addMotion(std::shared_ptr<Motion> motion) override;

    void solve() override;

    // simulation
    void
    setSimulationParameters(std::shared_ptr<SimulationParameters> simulationParameters) override;
    std::shared_ptr<SimulationParameters> getSimulationParameters() override;
    size_t numberOfFrames() override;
    void updateForFrame(size_t index) override;

    void preDrag() override;
    void dragStep(std::shared_ptr<std::vector<std::shared_ptr<Part>>> parts) override;
    void postDrag() override;

    void exportFile(std::string filename) override;

private:
    void setExternalSystem(AssemblyObject* assemblyObject);

    std::shared_ptr<SimulationParameters> simulationParameters;
    std::shared_ptr<MbD::ASMTAssembly> asmtAssembly;
};

class OndselSolver: public AssemblySolver
{
public:
    explicit OndselSolver(AssemblyObject* assemblyObject);
    virtual ~OndselSolver() = default;

    std::shared_ptr<Assembly> makeAssembly() override;

    std::shared_ptr<Part> makePart() override;

private:
    AssemblyObject* assemblyObject;
};

}  // namespace Assembly::Solver
