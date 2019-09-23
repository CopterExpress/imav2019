#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <functional>
#include <iostream>

// DANGER: OpenGL (ES) code ahead!

// Allow skipping pbuffer creation
#ifndef CREATE_PBUFFER
#define CREATE_PBUFFER 0
#endif /* CREATE_PBUFFER */

#define _XSTR(s) _STR(s)
#define _STR(s) #s

#define OGL_CHECKED(func) {GLenum error = glGetError(); if (error != GL_NO_ERROR) std::cerr << "OpenGL error at " << std::dec << __LINE__ << "while running" << _STR(func) << ": 0x" << std::hex << error << std::endl;}

namespace
{
    // This numer is chosen arbitrarily, we shouldn't really care about many configs
    const int MAX_NUM_CONFIG = 50;
}

namespace gles_undistort
{

class Undistorter : public nodelet::Nodelet
{
private:
    
    struct
    {
        EGLDisplay display = EGL_NO_DISPLAY;
        EGLConfig config[MAX_NUM_CONFIG];
        EGLContext context = EGL_NO_CONTEXT;
        EGLSurface surface = EGL_NO_SURFACE;
        EGLint numConfig;
        EGLint major, minor;
    } eglConnectionState;
    
    void createEglContext()
    {
        // Context attribute list - hardcoded for now
        // TODO: maybe not hardcode absolutely everything?
        EGLint attribList[] = {
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
        EGLint ctxAttribList[] = {
            EGL_CONTEXT_MAJOR_VERSION, 3, 
            EGL_CONTEXT_MINOR_VERSION, 0,
            EGL_NONE
        };

        // This fails for Raspberry Pi, but might just work for Jetson Nano (well, it did, actually). Usually one should pass a platform-native handle here
        eglConnectionState.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (eglConnectionState.display == EGL_NO_DISPLAY)
        {
            NODELET_WARN("No display connection available - expect problems");
        }
        EGLBoolean result = eglInitialize(eglConnectionState.display, &eglConnectionState.major, &eglConnectionState.minor);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Could not initialize EGL on default display; error %x", error);
            return;
        }
        NODELET_INFO("Initialized EGL version %d.%d", eglConnectionState.major, eglConnectionState.minor);
        result = eglChooseConfig(eglConnectionState.display, attribList, eglConnectionState.config, MAX_NUM_CONFIG, &eglConnectionState.numConfig);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Could not choose any appropriate config; error %x", error);
            return;
        }
        NODELET_INFO("Got %d configs. Using first one", eglConnectionState.numConfig);
        result = eglBindAPI(EGL_OPENGL_ES_API);
        if (result == EGL_FALSE)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Could not bind OpenGL ES api; error %x", error);
            return;
        }
        eglConnectionState.context = eglCreateContext(eglConnectionState.display, eglConnectionState.config[0], EGL_NO_CONTEXT, ctxAttribList);
        if (eglConnectionState.context == EGL_NO_CONTEXT)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Could not create context; error %x", error);
            return;
        }
#if CREATE_PBUFFER
        EGLint pbufferAttribs[] = {
            EGL_WIDTH, 640,
            EGL_HEIGHT, 480,
            EGL_NONE
        };
        eglConnectionState.surface = eglCreatePbufferSurface(eglConnectionState.display, eglConnectionState.config[0], pbufferAttribs);
        if (eglConnectionState.surface == EGL_NO_SURFACE)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Could not create surface; error %x", error);
        }
#endif
        // FIXME: check whether eglMakeCurrent succeeded?
        result = eglMakeCurrent(eglConnectionState.display, eglConnectionState.surface, eglConnectionState.surface, eglConnectionState.context);
        if (result != EGL_TRUE)
        {
            EGLint error = eglGetError();
            NODELET_ERROR("Somehow we could not set the context as current; error %x", error);
        }
        // Perform a simple smoke test. This typically fails hard if no context is available or a context is bad for some reason
        NODELET_INFO("Created EGL context. Vendor: %s, renderer: %s, version: %s", glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_VERSION));
        NODELET_INFO("Context handle is %p", eglConnectionState.context);
    }

    struct FboData
    {
        GLuint framebuffer;
        GLuint texture;
        GLsizei width;
        GLsizei height;
    };

    FboData createFbo(GLsizei width, GLsizei height)
    {
        FboData fbo{0, 0, 0, 0};
        glGenFramebuffers(1, &fbo.framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo.framebuffer);
        glGenTextures(1, &fbo.texture);
        NODELET_INFO("Generated framebuffer %d and texture %d", fbo.framebuffer, fbo.texture);
        glBindTexture(GL_TEXTURE_2D, fbo.texture);
        fbo.width = width;
        fbo.height = height;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, fbo.width, fbo.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo.texture, 0); OGL_CHECKED(glFramebufferTexture2D);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo.texture, 0); OGL_CHECKED(glFramebufferTexture2D);
        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE)
        {
            std::cerr << "Incomplete framebuffer, next operations will fail";
        }
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        return fbo;
    }

    void deleteFbo(FboData& data)
    {
        glDeleteFramebuffers(1, &data.framebuffer);
        glDeleteTextures(1, &data.texture);
        data.width = data.height = 0;
    }

    ros::NodeHandle nh_, nh_priv_;

    FboData testFbo;

    image_transport::Publisher fboPub;
    image_transport::CameraSubscriber camSub;

    ros::Timer timer_;
public:
    Undistorter()
    {
        
    }
    void onInit()
    {
        nh_ = getNodeHandle();
        nh_priv_ = getPrivateNodeHandle();
        image_transport::ImageTransport it_(nh_);
        image_transport::ImageTransport it_priv_(nh_priv_);

        //createEglContext();

        fboPub = it_priv_.advertise("image_rect", 1);

        camSub = it_.subscribeCamera("image_raw", 1, &Undistorter::cameraCallback, this);
    }

    float colR, colG, colB;

    void cameraCallback(const sensor_msgs::ImageConstPtr& src, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
    {
        if (eglGetCurrentContext() == nullptr)
        {
            NODELET_WARN("Recreating context!");
            createEglContext();
            testFbo = createFbo(src->width, src->height);
        }

        cv::Mat gl_image(testFbo.height, testFbo.width, CV_8UC4, cv::Scalar(0, 0, 255, 255));
        colR = (ros::Time::now().nsec) / 1e9f;
        colG = 1 - colR;
        colB = colR * colG;

        //NODELET_INFO("Publishing framebuffer %d, size (%d; %d)", testFbo.framebuffer, testFbo.width, testFbo.height);
        EGLContext ctx = eglGetCurrentContext();
        //NODELET_INFO("Using context handle %lx", ctx);

        glBindFramebuffer(GL_READ_FRAMEBUFFER, testFbo.framebuffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, testFbo.framebuffer);
        glViewport(0, 0, testFbo.width, testFbo.height);
        glClearColor(colR, colG, colB, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT); OGL_CHECKED(glClear);
        glPixelStorei(GL_PACK_ALIGNMENT, 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, gl_image.step / gl_image.elemSize());
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        
        glFlush();
        glFinish();
        glReadPixels(0, 0, testFbo.width, testFbo.height, GL_RGBA, GL_UNSIGNED_BYTE, gl_image.ptr()); OGL_CHECKED(glReadPixels);

        cv_bridge::CvImage img;
        img.image = gl_image;
        img.encoding = "bgra8";
        img.header.frame_id = "body";
        img.header.stamp = ros::Time::now();

        fboPub.publish(img.toImageMsg());
    }
};

}

PLUGINLIB_EXPORT_CLASS(gles_undistort::Undistorter, nodelet::Nodelet);