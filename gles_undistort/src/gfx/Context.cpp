#include "Context.h"
#include <stdexcept>
#include <sstream>

// Allow skipping pbuffer creation
#ifndef CREATE_PBUFFER
#define CREATE_PBUFFER 0
#endif /* CREATE_PBUFFER */

namespace Gfx
{
    EglConnectionState createContext()
    {
        // Context attribute list - hardcoded for now
        // TODO: maybe not hardcode absolutely everything?
        const EGLint attribList[] = {
            // Surface type: Jetson Nano wants a pbuffer for offscreen contexts, mesa works just fine without one. Note that we won't use pbuffer direc
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            // We want ES3 context to have something resembling a desktop OpenGL context
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
            // Request at least RGB888 for good measure
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_NONE
        };
        // Curiously enough, we have to also specify context attribs in a separate list
        const EGLint ctxAttribList[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3, 
            EGL_CONTEXT_MINOR_VERSION, 0,
            EGL_NONE
        };

        EglConnectionState ecs;

        // This fails for Raspberry Pi, but might just work for Jetson Nano (well, it did, actually). Usually one should pass a platform-native handle here
        ecs.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (ecs.display == EGL_NO_DISPLAY)
        {
            throw std::runtime_error("Cannot create display connection");
        }
        EGLBoolean result = eglInitialize(ecs.display, &ecs.major, &ecs.minor);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could not initialize EGL on default display; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
        result = eglChooseConfig(ecs.display, attribList, ecs.config, MAX_NUM_CONFIG, &ecs.numConfig);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could not choose any appropriate config; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
        result = eglBindAPI(EGL_OPENGL_ES_API);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could not bind EGL API to context; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
        ecs.context = eglCreateContext(ecs.display, ecs.config[0], EGL_NO_CONTEXT, ctxAttribList);
        if (ecs.context == EGL_NO_CONTEXT)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could create context; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
#if CREATE_PBUFFER
        EGLint pbufferAttribs[] = {
            EGL_WIDTH, 640,
            EGL_HEIGHT, 480,
            EGL_NONE
        };
        ecs.surface = eglCreatePbufferSurface(eglConnectionState.display, ecs.config[0], pbufferAttribs);
        if (ecs.surface == EGL_NO_SURFACE)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could not create Pbuffer surface; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
#endif
        // FIXME: check whether eglMakeCurrent succeeded?
        result = eglMakeCurrent(ecs.display, ecs.surface, ecs.surface, ecs.context);
        if (result != EGL_TRUE)
        {
            EGLint error = eglGetError();
            std::stringstream ss;
            ss << "Could not set context as current; error 0x" << std::hex << error;
            throw std::runtime_error(ss.str());
        }
        return ecs;
    }

    FboData createFbo(GLsizei width, GLsizei height)
    {
        FboData fbo{0, 0, 0, 0};
        glGenFramebuffers(1, &fbo.framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo.framebuffer);
        glGenTextures(1, &fbo.texture);
        glBindTexture(GL_TEXTURE_2D, fbo.texture);
        fbo.width = width;
        fbo.height = height;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, fbo.width, fbo.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo.texture, 0);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo.texture, 0);
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            throw std::runtime_error("Incomplete framebuffer created - cannot use for drawing");
        }
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        return fbo;
    }

    void deleteFbo(FboData& fbo)
    {
        glDeleteFramebuffers(1, &fbo.framebuffer);
        glDeleteTextures(1, &fbo.texture);
        fbo.width = fbo.height = 0;
    }
}
