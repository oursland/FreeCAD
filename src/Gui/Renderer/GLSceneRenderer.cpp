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
# include <OpenGL/gl.h>
# include <OpenGL/glext.h>
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

// GLSL 1.20 shaders for compatibility with macOS's OpenGL 2.1 compatibility profile.
// (macOS only exposes GL 2.1 / GLSL 1.20 in CompatibilityProfile; Core Profile
// would give 4.1 but would break Coin3D's fixed-function rendering.)

static const char* vertexShaderSrc = R"(
#version 120
attribute vec3 aPos;
attribute vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;
uniform mat4 uModel;

varying vec3 vNormal;
varying vec3 vFragPos;

void main()
{
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vFragPos = worldPos.xyz;
    // Normal matrix: transpose(inverse(modelMatrix))
    // GLSL 1.20 has no inverse(), so we approximate for uniform scaling
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uProj * uView * worldPos;
}
)";

static const char* fragmentShaderSrc = R"(
#version 120
varying vec3 vNormal;
varying vec3 vFragPos;

uniform vec4 uColor;          // rgb + alpha
uniform vec3 uLightDir;       // directional light in world space
uniform vec4 uOverrideColor;  // selection/highlight override
uniform int  uUseOverride;    // 1 = use override color

void main()
{
    vec3 baseColor = (uUseOverride != 0) ? uOverrideColor.rgb : uColor.rgb;
    float alpha = (uUseOverride != 0) ? uOverrideColor.a : uColor.a;

    vec3 norm = normalize(vNormal);
    float diff = max(dot(norm, normalize(uLightDir)), 0.0);
    float ambient = 0.3;
    vec3 result = baseColor * (ambient + diff * 0.7);

    gl_FragColor = vec4(result, alpha);
}
)";

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static void sbMatrixToFloat16(const SbMatrix& m, std::array<float, 16>& out)
{
    // SbMatrix is already in OpenGL column-major layout (Coin3D passes it
    // directly to glLoadMatrixf). Just copy the 16 floats.
    std::memcpy(out.data(), m[0], 16 * sizeof(float));
}

// -----------------------------------------------------------------------
// GLSceneRenderer implementation
// -----------------------------------------------------------------------

GLSceneRenderer::GLSceneRenderer() = default;

GLSceneRenderer::~GLSceneRenderer()
{
    // Clean up GL buffers
    for (auto& entry : queue.entries()) {
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
        Base::Console().error("GLSceneRenderer: failed to build mesh shader\n");
        return false;
    }

    // Bind attribute locations (GLSL 1.20 doesn't support layout qualifiers)
    meshShader.bindAttribAndRelink({{0, "aPos"}, {1, "aNormal"}});

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
    Base::Console().log("GLSceneRenderer: initialized successfully\n");
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
    // Disable face culling for now — Coin's view matrix may flip winding
    glDisable(GL_CULL_FACE);

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

    static bool loggedOnce = false;
    if (!loggedOnce && !queue.entries().empty()) {
        Base::Console().log(
            "GLSceneRenderer: rendering %d entries\n",
            static_cast<int>(queue.entries().size())
        );
        const auto& first = queue.entries()[0];
        Base::Console().log(
            "  first entry: vbo=%u ebo=%u elements=%d type=%d\n",
            first.vbo,
            first.ebo,
            first.numElements,
            static_cast<int>(first.type)
        );
        loggedOnce = true;
    }

    // Pass 1: Opaque geometry
    glDisable(GL_BLEND);
    for (const auto& entry : queue.entries()) {
        if (!entry.visible || !entry.vbo) {
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
        if (!entry.visible || !entry.vbo) {
            continue;
        }
        if (entry.transparency <= 0.001f) {
            continue;
        }
        renderEntry(entry);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    // Restore GL state for Coin3D
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
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
    // Apply selection/preselection override color
    if (entry.highlightElement >= 0) {
        meshShader.setUniform4f(
            uOverrideColor,
            entry.highlightColor[0],
            entry.highlightColor[1],
            entry.highlightColor[2],
            1.0f
        );
        meshShader.setUniform1i(uUseOverride, 1);
    }
    else if (!entry.selectedElements.empty()) {
        meshShader.setUniform4f(
            uOverrideColor,
            entry.selectionColor[0],
            entry.selectionColor[1],
            entry.selectionColor[2],
            1.0f
        );
        meshShader.setUniform1i(uUseOverride, 1);
    }
    else {
        meshShader.setUniform1i(uUseOverride, 0);
    }

    // Bind VBO and set vertex attribute pointers (no VAO in GL 2.1)
    int stride = 6 * sizeof(float);  // 3 pos + 3 normal interleaved
    glBindBuffer(GL_ARRAY_BUFFER, entry.vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));

    if (entry.ebo) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, entry.ebo);
    }

    switch (entry.type) {
        case DrawType::Triangles:
            glDrawElements(GL_TRIANGLES, entry.numElements, GL_UNSIGNED_INT, nullptr);
            break;
        case DrawType::Lines:
            glLineWidth(entry.lineWidth);
            glDrawElements(GL_LINES, entry.numElements, GL_UNSIGNED_INT, nullptr);
            break;
        case DrawType::Points:
            glPointSize(entry.pointSize);
            glDrawArrays(GL_POINTS, 0, entry.numElements);
            break;
    }

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
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
    entry.vao = 0;  // Not using VAOs (GL 2.1 compatibility)

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
    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
        interleaved.data(),
        GL_STATIC_DRAW
    );
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    if (indices && numIndices > 0) {
        glGenBuffers(1, &entry.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, entry.ebo);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(numIndices * sizeof(int32_t)),
            indices,
            GL_STATIC_DRAW
        );
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        entry.numElements = numIndices;
    }
    else {
        entry.numElements = numVerts;
    }

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
