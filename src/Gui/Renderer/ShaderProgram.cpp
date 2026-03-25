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

#include <FCConfig.h>

#ifndef FC_OS_WIN32
# ifndef GL_GLEXT_PROTOTYPES
#  define GL_GLEXT_PROTOTYPES 1
# endif
#else
# include <windows.h>
#endif

#ifdef FC_OS_MACOSX
# include <OpenGL/gl3.h>
#else
# include <GL/gl.h>
# include <GL/glext.h>
#endif

#include "ShaderProgram.h"
#include <Base/Console.h>

using namespace Gui;

ShaderProgram::ShaderProgram() = default;

ShaderProgram::~ShaderProgram()
{
    if (program) {
        glDeleteProgram(program);
    }
}

uint32_t ShaderProgram::compileShader(uint32_t type, const char* src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        Base::Console().Error("ShaderProgram: compile error: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool ShaderProgram::build(const char* vertexSrc, const char* fragmentSrc)
{
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
    if (!vs) {
        return false;
    }

    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
    if (!fs) {
        glDeleteShader(vs);
        return false;
    }

    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        Base::Console().Error("ShaderProgram: link error: %s\n", log);
        glDeleteProgram(program);
        program = 0;
    }

    // Shaders can be deleted after linking
    glDeleteShader(vs);
    glDeleteShader(fs);

    return program != 0;
}

void ShaderProgram::use() const
{
    glUseProgram(program);
}

int ShaderProgram::uniformLocation(const char* name) const
{
    return glGetUniformLocation(program, name);
}

void ShaderProgram::setUniformMatrix4fv(int location, const float* value) const
{
    glUniformMatrix4fv(location, 1, GL_FALSE, value);
}

void ShaderProgram::setUniform3f(int location, float x, float y, float z) const
{
    glUniform3f(location, x, y, z);
}

void ShaderProgram::setUniform4f(int location, float x, float y, float z, float w) const
{
    glUniform4f(location, x, y, z, w);
}

void ShaderProgram::setUniform1f(int location, float value) const
{
    glUniform1f(location, value);
}

void ShaderProgram::setUniform1i(int location, int value) const
{
    glUniform1i(location, value);
}
