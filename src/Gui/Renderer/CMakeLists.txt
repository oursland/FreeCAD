set(Renderer_SRCS
    SceneRenderer.h
    ShaderProgram.h
    ShaderProgram.cpp
    RenderQueue.h
    RenderQueue.cpp
    GLSceneRenderer.h
    GLSceneRenderer.cpp
    SceneSync.h
    SceneSync.cpp
)

# These files are compiled as part of the FreeCADGui library,
# not as a separate library.
set(FreeCADGui_Renderer_SRCS ${Renderer_SRCS} PARENT_SCOPE)
