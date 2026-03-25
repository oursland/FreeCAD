/***************************************************************************
 *   Copyright (c) 2026 FreeCAD Project Association                        *
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
 *   License along with this library; see the file COPYING.LIB. If not,   *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace Gui
{

/// Minimal OpenGL shader program wrapper.
/// Compiles vertex + fragment shaders, manages uniform locations.
class ShaderProgram
{
public:
    ShaderProgram();
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    /// Compile and link from source strings.  Returns true on success.
    bool build(const char* vertexSrc, const char* fragmentSrc);

    /// Bind attribute locations before linking.  Call before build(), or
    /// call build() first and then re-link with bindAttribAndRelink().
    void bindAttribAndRelink(const std::vector<std::pair<uint32_t, const char*>>& attribs);

    /// Activate this program for subsequent draw calls.
    void use() const;

    /// Get uniform location by name (cached internally).
    int uniformLocation(const char* name) const;

    /// Set uniform values.
    void setUniformMatrix4fv(int location, const float* value) const;
    void setUniform3f(int location, float x, float y, float z) const;
    void setUniform4f(int location, float x, float y, float z, float w) const;
    void setUniform1f(int location, float value) const;
    void setUniform1i(int location, int value) const;

    uint32_t id() const
    {
        return program;
    }

private:
    uint32_t program = 0;

    static uint32_t compileShader(uint32_t type, const char* src);
};

}  // namespace Gui
