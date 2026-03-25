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

#include "GLSceneRenderer.h"

#include <Inventor/SbMatrix.h>
#include <Inventor/SbColor.h>
#include <Inventor/SbViewportRegion.h>
#include <Base/Console.h>

#include <cstring>

using namespace Gui;

// -----------------------------------------------------------------------
// Shaders
// -----------------------------------------------------------------------

static const char* vertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;
uniform mat4 uModel;

out vec3 vNormal;
out vec3 vFragPos;

void main()
{
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vFragPos = worldPos.xyz;
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uProj * uView * worldPos;
}
)";

static const char* fragmentShaderSrc = R"(
#version 330 core
in vec3 vNormal;
in vec3 vFragPos;

uniform vec4 uColor;          // rgb + alpha
uniform vec3 uLightDir;       // directional light in world space
uniform vec4 uOverrideColor;  // selection/highlight override
uniform int  uUseOverride;    // 1 = use override color

out vec4 FragColor;

void main()
{
    vec3 baseColor = (uUseOverride != 0) ? uOverrideColor.rgb : uColor.rgb;
    float alpha = (uUseOverride != 0) ? uOverrideColor.a : uColor.a;

    vec3 norm = normalize(vNormal);
    float diff = max(dot(norm, normalize(uLightDir)), 0.0);
    float ambient = 0.3;
    vec3 result = baseColor * (ambient + diff * 0.7);

    FragColor = vec4(result, alpha);
}
)";

// Flat shader for lines and points (no lighting)
static const char* flatFragmentShaderSrc = R"(
#version 330 core
in vec3 vNormal;
in vec3 vFragPos;

uniform vec4 uColor;
uniform vec4 uOverrideColor;
uniform int  uUseOverride;

out vec4 FragColor;

void main()
{
    vec3 baseColor = (uUseOverride != 0) ? uOverrideColor.rgb : uColor.rgb;
    float alpha = (uUseOverride != 0) ? uOverrideColor.a : uColor.a;
    FragColor = vec4(baseColor, alpha);
}
)";

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static void sbMatrixToFloat16(const SbMatrix& m, std::array<float, 16>& out)
{
    // SbMatrix stores row-major, OpenGL expects column-major
    const float* src = m[0];
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            out[col * 4 + row] = src[row * 4 + col];
        }
    }
}

// -----------------------------------------------------------------------
// GLSceneRenderer implementation
// -----------------------------------------------------------------------

GLSceneRenderer::GLSceneRenderer() = default;

GLSceneRenderer::~GLSceneRenderer()
{
    // Clean up GL buffers
    for (auto& entry : queue.entries()) {
        if (entry.vao) {
            glDeleteVertexArrays(1, &entry.vao);
        }
        if (entry.vbo) {
            glDeleteBuffers(1, &entry.vbo);
        }
        if (entry.ebo) {
            glDeleteBuffers(1, &entry.ebo);
        }
    }
}

bool GLSceneRenderer::initialize()
{
    if (initialized) {
        return true;
    }

    if (!meshShader.build(vertexShaderSrc, fragmentShaderSrc)) {
        Base::Console().Error("GLSceneRenderer: failed to build mesh shader\n");
        return false;
    }

    // Cache uniform locations
    meshShader.use();
    uViewMatrix = meshShader.uniformLocation("uView");
    uProjMatrix = meshShader.uniformLocation("uProj");
    uModelMatrix = meshShader.uniformLocation("uModel");
    uColor = meshShader.uniformLocation("uColor");
    uLightDir = meshShader.uniformLocation("uLightDir");
    uOverrideColor = meshShader.uniformLocation("uOverrideColor");
    uUseOverride = meshShader.uniformLocation("uUseOverride");

    initialized = true;
    Base::Console().Log("GLSceneRenderer: initialized successfully\n");
    return true;
}

void GLSceneRenderer::beginFrame(const SbMatrix& view, const SbMatrix& proj, const SbViewportRegion& viewport)
{
    if (!initialized) {
        return;
    }

    sbMatrixToFloat16(view, viewMatrix);
    sbMatrixToFloat16(proj, projMatrix);

    auto size = viewport.getViewportSizePixels();
    glViewport(0, 0, size[0], size[1]);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    meshShader.use();
    meshShader.setUniformMatrix4fv(uViewMatrix, viewMatrix.data());
    meshShader.setUniformMatrix4fv(uProjMatrix, projMatrix.data());
    meshShader.setUniform3f(uLightDir, 0.3f, 0.5f, 0.8f);

    queue.sort();
}

void GLSceneRenderer::endFrame()
{
    if (!initialized) {
        return;
    }

    // Pass 1: Opaque geometry
    glDisable(GL_BLEND);
    for (const auto& entry : queue.entries()) {
        if (!entry.visible) {
            continue;
        }
        if (entry.transparency > 0.001f) {
            continue;  // skip transparent
        }
        renderEntry(entry);
    }

    // Pass 2: Transparent geometry (back-to-front via sort key)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const auto& entry : queue.entries()) {
        if (!entry.visible) {
            continue;
        }
        if (entry.transparency <= 0.001f) {
            continue;
        }
        renderEntry(entry);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void GLSceneRenderer::renderEntry(const DrawEntry& entry)
{
    meshShader.setUniformMatrix4fv(uModelMatrix, entry.modelMatrix.data());
    meshShader.setUniform4f(
        uColor,
        entry.color[0],
        entry.color[1],
        entry.color[2],
        1.0f - entry.transparency
    );
    meshShader.setUniform1i(uUseOverride, 0);

    glBindVertexArray(entry.vao);

    switch (entry.type) {
        case DrawType::Triangles:
            glDrawElements(GL_TRIANGLES, entry.numElements, GL_UNSIGNED_INT, nullptr);
            break;
        case DrawType::Lines:
            glLineWidth(entry.lineWidth);
            glDrawElements(GL_LINES, entry.numElements, GL_UNSIGNED_INT, nullptr);
            break;
        case DrawType::Points:
#ifndef FC_OS_MACOSX
            glPointSize(entry.pointSize);
#endif
            glDrawArrays(GL_POINTS, 0, entry.numElements);
            break;
    }

    glBindVertexArray(0);
}

// -----------------------------------------------------------------------
// Geometry submission
// -----------------------------------------------------------------------

DrawEntry GLSceneRenderer::createGLBuffers(
    DrawType type,
    const float* vertices,
    int numVerts,
    const float* normals,
    const int32_t* indices,
    int numIndices
)
{
    DrawEntry entry;
    entry.type = type;

    glGenVertexArrays(1, &entry.vao);
    glBindVertexArray(entry.vao);

    // Interleave position + normal into a single VBO
    int stride = 6;  // 3 pos + 3 normal
    std::vector<float> interleaved(numVerts * stride);
    for (int i = 0; i < numVerts; i++) {
        interleaved[i * stride + 0] = vertices[i * 3 + 0];
        interleaved[i * stride + 1] = vertices[i * 3 + 1];
        interleaved[i * stride + 2] = vertices[i * 3 + 2];
        if (normals) {
            interleaved[i * stride + 3] = normals[i * 3 + 0];
            interleaved[i * stride + 4] = normals[i * 3 + 1];
            interleaved[i * stride + 5] = normals[i * 3 + 2];
        }
        else {
            interleaved[i * stride + 3] = 0.0f;
            interleaved[i * stride + 4] = 0.0f;
            interleaved[i * stride + 5] = 1.0f;
        }
    }

    glGenBuffers(1, &entry.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, entry.vbo);
    glBufferData(GL_ARRAY_BUFFER, interleaved.size() * sizeof(float), interleaved.data(), GL_STATIC_DRAW);

    // Position attribute (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute (location 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    if (indices && numIndices > 0) {
        glGenBuffers(1, &entry.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, entry.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * sizeof(int32_t), indices, GL_STATIC_DRAW);
        entry.numElements = numIndices;
    }
    else {
        entry.numElements = numVerts;
    }

    glBindVertexArray(0);
    return entry;
}

SceneRenderer::MeshId GLSceneRenderer::submitMesh(
    const float* vertices,
    int numVerts,
    const int32_t* indices,
    int numIndices,
    const float* normals,
    const SbMatrix& transform,
    const SbColor& color,
    float transparency
)
{
    DrawEntry entry
        = createGLBuffers(DrawType::Triangles, vertices, numVerts, normals, indices, numIndices);
    entry.meshId = nextId++;
    sbMatrixToFloat16(transform, entry.modelMatrix);
    color.getValue(entry.color[0], entry.color[1], entry.color[2]);
    entry.transparency = transparency;

    queue.addOrUpdate(entry);
    return entry.meshId;
}

SceneRenderer::MeshId GLSceneRenderer::submitLines(
    const float* vertices,
    int numVerts,
    const int32_t* indices,
    int numIndices,
    const SbMatrix& transform,
    const SbColor& color,
    float lineWidth
)
{
    DrawEntry entry
        = createGLBuffers(DrawType::Lines, vertices, numVerts, nullptr, indices, numIndices);
    entry.meshId = nextId++;
    sbMatrixToFloat16(transform, entry.modelMatrix);
    color.getValue(entry.color[0], entry.color[1], entry.color[2]);
    entry.lineWidth = lineWidth;

    queue.addOrUpdate(entry);
    return entry.meshId;
}

SceneRenderer::MeshId GLSceneRenderer::submitPoints(
    const float* vertices,
    int numVerts,
    const SbMatrix& transform,
    const SbColor& color,
    float pointSize
)
{
    DrawEntry entry = createGLBuffers(DrawType::Points, vertices, numVerts, nullptr, nullptr, 0);
    entry.meshId = nextId++;
    sbMatrixToFloat16(transform, entry.modelMatrix);
    color.getValue(entry.color[0], entry.color[1], entry.color[2]);
    entry.pointSize = pointSize;

    queue.addOrUpdate(entry);
    return entry.meshId;
}

// -----------------------------------------------------------------------
// Updates
// -----------------------------------------------------------------------

void GLSceneRenderer::updateTransform(MeshId id, const SbMatrix& transform)
{
    if (auto* entry = queue.find(id)) {
        sbMatrixToFloat16(transform, entry->modelMatrix);
    }
}

void GLSceneRenderer::updateMaterial(MeshId id, const SbColor& color, float transparency)
{
    if (auto* entry = queue.find(id)) {
        color.getValue(entry->color[0], entry->color[1], entry->color[2]);
        entry->transparency = transparency;
    }
}

void GLSceneRenderer::setVisible(MeshId id, bool visible)
{
    if (auto* entry = queue.find(id)) {
        entry->visible = visible;
    }
}

void GLSceneRenderer::remove(MeshId id)
{
    if (auto* entry = queue.find(id)) {
        if (entry->vao) {
            glDeleteVertexArrays(1, &entry->vao);
        }
        if (entry->vbo) {
            glDeleteBuffers(1, &entry->vbo);
        }
        if (entry->ebo) {
            glDeleteBuffers(1, &entry->ebo);
        }
    }
    queue.remove(id);
}

// -----------------------------------------------------------------------
// Selection / highlighting
// -----------------------------------------------------------------------

void GLSceneRenderer::setHighlight(MeshId id, int elementIndex, const SbColor& color)
{
    if (auto* entry = queue.find(id)) {
        entry->highlightElement = elementIndex;
        color.getValue(entry->highlightColor[0], entry->highlightColor[1], entry->highlightColor[2]);
    }
}

void GLSceneRenderer::clearHighlight(MeshId id)
{
    if (auto* entry = queue.find(id)) {
        entry->highlightElement = -1;
    }
}

void GLSceneRenderer::setSelection(MeshId id, const std::vector<int>& elementIndices, const SbColor& color)
{
    if (auto* entry = queue.find(id)) {
        entry->selectedElements = elementIndices;
        color.getValue(entry->selectionColor[0], entry->selectionColor[1], entry->selectionColor[2]);
    }
}

void GLSceneRenderer::clearSelection(MeshId id)
{
    if (auto* entry = queue.find(id)) {
        entry->selectedElements.clear();
    }
}

// -----------------------------------------------------------------------
// Invalidation
// -----------------------------------------------------------------------

void GLSceneRenderer::invalidate(MeshId id)
{
    // For now, geometry re-upload is handled by remove + re-submit.
    // A smarter implementation would flag the entry and re-upload on next frame.
    (void)id;
}
