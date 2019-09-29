/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#ifndef ZBAR_ROS_ZBAR_ROS_NODELET_H
#define ZBAR_ROS_ZBAR_ROS_NODELET_H

#include "ros/ros.h"
#include "opencv/cv.h"
#include "cv_bridge/cv_bridge.h"
#include "zbar.h"
#include <unordered_map>
#include <string>
#include <vector>
#include "nodelet/nodelet.h"
#include <image_transport/image_transport.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

namespace zbar_ros_redux
{

  class BarcodeReaderNodelet : public nodelet::Nodelet
  {
  public:
    BarcodeReaderNodelet();

  private:
    virtual void onInit();
    void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr& cinfo, cv::Mat& matrix, cv::Mat& dist);
    void imageCb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr &cinfo);
    ros::NodeHandle nh_, private_nh_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::Publisher debug_pub_;
    ros::Publisher qr_pub_;
    ros::Timer clean_timer_;
    zbar::ImageScanner scanner_;
    cv::Mat camera_matrix_, dist_coeffs_;
    double symbol_size_;
    bool send_tf_;
    std::vector<cv::Point3f> obj_points_;
    tf2_ros::TransformBroadcaster br_;
  };

}  // namespace zbar_ros

#endif  // ZBAR_ROS_ZBAR_ROS_NODELET_H
