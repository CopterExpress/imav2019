#include "gfx/Context.h"
#include "gfx/UndistortedMesh.h"
#include "gfx/Texture.h"
#include "gfx/Shader.h"
#include "gfx/ShaderSources.h"

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

#define _XSTR(s) _STR(s)
#define _STR(s) #s

#define OGL_CHECKED(func) {GLenum error = glGetError(); if (error != GL_NO_ERROR) std::cerr << "OpenGL error at " << std::dec << __LINE__ << "while running" << _STR(func) << ": 0x" << std::hex << error << std::endl;}

namespace gles_undistort
{

class Undistorter : public nodelet::Nodelet
{
private:

    Gfx::EglConnectionState ecs_;

    Gfx::FboData fbo_;
    Gfx::TextureData tex_;
    Gfx::GpuMesh mesh_;
    Gfx::Shader shader_;

    ros::NodeHandle nh_, nh_priv_;

    image_transport::CameraPublisher fbo_pub_;
    image_transport::CameraSubscriber cam_sub_;

    int mesh_subdiv_x_;
    int mesh_subdiv_y_;

    sensor_msgs::CameraInfoPtr camera_info_;

    ros::Timer timer_;
public:
    Undistorter() {}
    void onInit()
    {
        nh_ = getNodeHandle();
        nh_priv_ = getPrivateNodeHandle();
        image_transport::ImageTransport it_(nh_);
        image_transport::ImageTransport it_priv_(nh_priv_);

        //createEglContext();

        mesh_subdiv_x_ = nh_priv_.param("subdiv_x", 40);
        mesh_subdiv_y_ = nh_priv_.param("subdiv_y", 30);

        camera_info_ = boost::make_shared<sensor_msgs::CameraInfo>();

        fbo_pub_ = it_priv_.advertiseCamera("image_rect", 1);

        cam_sub_ = it_.subscribeCamera("image_raw", 1, &Undistorter::cameraCallback, this);
    }

    void cameraCallback(const sensor_msgs::ImageConstPtr& src, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
    {
        if (ecs_.context == EGL_NO_CONTEXT)
        {
            NODELET_INFO("Creating EGL context");
            try
            {
                ecs_ = Gfx::createContext();
                fbo_ = Gfx::createFbo(src->width, src->height);
            }
            catch (std::runtime_error &e)
            {
                NODELET_FATAL("Could not obtain EGL context! Failed with error: %s", e.what());
                return;
            }
            NODELET_INFO("Created EGL context version %d.%d", ecs_.major, ecs_.minor);
            NODELET_INFO("GL vendor: %s; GL renderer: %s; GL version: %s", glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_VERSION));
            // FIXME: move all of that to a different function?
            // FIXME: This does not react to changes
            tex_ = Gfx::createTexture(src->width, src->height, nullptr);

            NODELET_INFO("Creating regular mesh");
            // FIXME: Add vertices count as parameter
            Gfx::Mesh origMesh = Gfx::createRegularMesh(mesh_subdiv_x_, mesh_subdiv_y_, src->width, src->height);
            NODELET_INFO("Creating undistortion mesh");
            Gfx::Mesh undistMesh = Gfx::createUndistortedMesh(origMesh, cameraInfo);

            // Normalize vertex positions to [-1, 1] on both axes
            for(auto& vtx : undistMesh.vertices)
            {
                vtx.vpos[0] = (2.0f * vtx.vpos[0] / src->width) - 1.0f;
                vtx.vpos[1] = (2.0f * vtx.vpos[1] / src->height) - 1.0f;
            }
            NODELET_INFO("Uploading mesh to GPU");
            mesh_ = Gfx::uploadMesh(undistMesh);
            shader_.init().addStage(GL_VERTEX_SHADER, Gfx::Shaders::undistortVertexShader).addStage(GL_FRAGMENT_SHADER, Gfx::Shaders::undistortFragmentShader).link();
        }
        if (ecs_.context != eglGetCurrentContext())
        {
            NODELET_ERROR("Callback called from a different thread, cannot use current EGL context");
            return;
        }

        const auto src_image = cv_bridge::toCvShare(src, "bgra8");
        tex_ = Gfx::updateTexture(tex_, src->width, src->height, src_image->image.data);

        cv::Mat gl_image(fbo_.height, fbo_.width, CV_8UC4, cv::Scalar(0, 0, 255, 255));

        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_.framebuffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_.framebuffer);
        glViewport(0, 0, fbo_.width, fbo_.height);
        glClear(GL_COLOR_BUFFER_BIT);
        render(mesh_, shader_, tex_.id);
        glPixelStorei(GL_PACK_ALIGNMENT, 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, gl_image.step / gl_image.elemSize());
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, fbo_.width, fbo_.height, GL_RGBA, GL_UNSIGNED_BYTE, gl_image.ptr()); OGL_CHECKED(glReadPixels);

        cv_bridge::CvImage img;
        img.image = gl_image;
        img.encoding = "bgra8";
        img.header.frame_id = "body";
        img.header.stamp = src->header.stamp;

        // Create new camera_info message without distortion
        camera_info_->header.frame_id = cameraInfo->header.frame_id;
        camera_info_->header.stamp = cameraInfo->header.stamp;
        camera_info_->width = cameraInfo->width;
        camera_info_->height = cameraInfo->height;
        camera_info_->K = cameraInfo->K;
        camera_info_->D = std::vector<double>(5);
        camera_info_->P = cameraInfo->P;
        camera_info_->R = cameraInfo->R;
        camera_info_->distortion_model = cameraInfo->distortion_model;
        camera_info_->binning_x = cameraInfo->binning_x;
        camera_info_->binning_y = cameraInfo->binning_y;
        camera_info_->roi = cameraInfo->roi;

        fbo_pub_.publish(img.toImageMsg(), camera_info_);
    }

    static void render(const Gfx::GpuMesh& mesh, const Gfx::Shader& shader, GLuint texture)
    {
        shader.activate();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture);

        glUniform1i(shader.unfLoc("tex"), 0);

        GLint posLoc = shader.attrLoc("pos");
        GLint texLoc = shader.attrLoc("texCoord");
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vtxBuf);
        glEnableVertexAttribArray(posLoc);
        glVertexAttribPointer(posLoc, 2, GL_FLOAT, GL_FALSE, sizeof(Gfx::VertexData), (void*)offsetof(Gfx::VertexData, vpos));
        glEnableVertexAttribArray(texLoc);
        glVertexAttribPointer(texLoc, 2, GL_FLOAT, GL_FALSE, sizeof(Gfx::VertexData), (void*)offsetof(Gfx::VertexData, texcoord));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.idxBuf);
        glDrawElements(GL_TRIANGLES, mesh.idxCount, GL_UNSIGNED_INT, nullptr);
}

};

}

PLUGINLIB_EXPORT_CLASS(gles_undistort::Undistorter, nodelet::Nodelet);