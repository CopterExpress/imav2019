#pragma once

#include <EGL/egl.h>
#include <GLES3/gl3.h>

namespace Gfx
{
    // This numer is chosen arbitrarily, we shouldn't really care about many configs
    const int MAX_NUM_CONFIG = 50;
    struct EglConnectionState
    {
        EGLDisplay display = EGL_NO_DISPLAY;
        EGLConfig config[MAX_NUM_CONFIG];
        EGLContext context = EGL_NO_CONTEXT;
        EGLSurface surface = EGL_NO_SURFACE;
        EGLint numConfig = -1;
        EGLint major = -1, minor = -1;
    };

    struct FboData
    {
        GLuint framebuffer;
        GLuint texture;
        GLsizei width;
        GLsizei height;
    };

    /// Create EGL offscreen context
    EglConnectionState createContext();

    /// Create a framebuffer object
    FboData createFbo(GLsizei width, GLsizei height);

    /// Destroy a framebuffer object
    void deleteFbo(FboData& fbo);
}
