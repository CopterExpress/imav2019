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

#include "zbar_ros_redux/barcode_reader_nodelet.h"
#include "zbar_ros_redux/DetectedQr.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/String.h"
#include <functional>
#include <opencv2/opencv.hpp>

namespace zbar_ros_redux
{

  BarcodeReaderNodelet::BarcodeReaderNodelet()
  {
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  void BarcodeReaderNodelet::onInit()
  {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh_);
    image_transport::ImageTransport it_priv(private_nh_);

    camera_sub_ = it.subscribe("image_raw", 1, &BarcodeReaderNodelet::imageCb, this);
    debug_pub_ = it_priv.advertise("debug", 1);

    qr_pub_ = private_nh_.advertise<zbar_ros_redux::DetectedQr>("qr", 1);
  }

  void BarcodeReaderNodelet::imageCb(const sensor_msgs::ImageConstPtr &image)
  {
    bool publish_debug = debug_pub_.getNumSubscribers() > 0;
    bool has_subscribers = (qr_pub_.getNumSubscribers() > 0) || publish_debug;
    if (!has_subscribers)
    {
      return;
    }
    
    cv_bridge::CvImageConstPtr cv_image;
    cv_bridge::CvImagePtr debug_image;
    cv_image = cv_bridge::toCvShare(image, "mono8");
    if (publish_debug)
    {
      debug_image = cv_bridge::toCvCopy(image);
    }

    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
        cv_image->image.cols * cv_image->image.rows);
    scanner_.scan(zbar_image);

    // iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol)
    {
      std::string barcode = symbol->get_data();
      NODELET_INFO("Found symbol: %s, bounding box has %d entries", barcode.c_str(), symbol->get_location_size());

      // publish barcode
      zbar_ros_redux::DetectedQr detected_message;
      detected_message.header.frame_id = image->header.frame_id;
      detected_message.header.stamp = image->header.stamp;
      detected_message.qr_message = barcode;
      for(int i = 0; i < symbol->get_location_size(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = symbol->get_location_x(i);
        pt.y = symbol->get_location_y(i);
        pt.z = 0;
        detected_message.bounding_polygon.push_back(pt);
      }
      qr_pub_.publish(detected_message);
      if (publish_debug)
      {
        int pt_count = symbol->get_location_size();
        cv::Point2d centroid;
        for(int i = 0; i < pt_count; ++i)
        {
          cv::Point2d prev_point{symbol->get_location_x(i), symbol->get_location_y(i)};
          cv::Point2d cur_point{symbol->get_location_x((i + 1) % pt_count), symbol->get_location_y((i + 1) % pt_count)};
          cv::line(debug_image->image, prev_point, cur_point, cv::Scalar(0, 255, 0));
          centroid += prev_point;
        }
        centroid.x /= pt_count;
        centroid.y /= pt_count;
        cv::putText(debug_image->image, barcode, centroid, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 0));
      }
    }
    if (publish_debug)
    {
      debug_pub_.publish(debug_image->toImageMsg());
    }

    zbar_image.set_data(NULL, 0);
  }

}  // namespace zbar_ros

PLUGINLIB_EXPORT_CLASS(zbar_ros_redux::BarcodeReaderNodelet, nodelet::Nodelet);
