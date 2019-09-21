#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <functional>

// DANGER: OpenGL (ES) code ahead!

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
        EGLContext context;
        EGLSurface surface;
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
        // FIXME: check whether eglMakeCurrent succeeded?
        eglMakeCurrent(eglConnectionState.display, EGL_NO_SURFACE, EGL_NO_SURFACE, eglConnectionState.context);
        // Perform a simple smoke test. This typically fails hard if no context is available or a context is bad for some reason
        NODELET_INFO("Created EGL context. Vendor: %s, renderer: %s, version: %s", glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_VERSION));
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
        FboData fbo;
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
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
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

        createEglContext();
        
        testFbo = createFbo(640, 480);

        fboPub = it_priv_.advertise("image_rect", 1);

        timer_ = nh_.createTimer(ros::Duration(10), std::bind(&Undistorter::publishFbo, this, std::placeholders::_1));


    }

    void publishFbo(const ros::TimerEvent& event)
    {


    }

};

}

PLUGINLIB_EXPORT_CLASS(gles_undistort::Undistorter, nodelet::Nodelet);