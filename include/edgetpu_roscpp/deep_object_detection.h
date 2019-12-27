// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once
#include <algorithm>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include <cv_bridge/cv_bridge.h>
#include "edgetpu_roscpp/image_resize.h"
#include <image_transport/image_transport.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "src/cpp/detection/engine.h"
#include "src/cpp/examples/label_utils.h"
#include "src/cpp/examples/model_utils.h"

namespace edgetpu_roscpp
{
  class DeepObjectDetection: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    DeepObjectDetection(): DiagnosticNodelet("DeepObjectDetection"), model_tensor_shape_(0) {}

  protected:
    /* ros publisher */
    //ros::Publisher target_pos_pub_;
    image_transport::Publisher image_pub_; //for debug

    /* ros subscriber */
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;

    /* image transport */
    boost::shared_ptr<image_transport::ImageTransport> it_;

    /* detection */
    boost::shared_ptr<coral::DetectionEngine> detection_engine_;
    std::vector<int> model_tensor_shape_;
    std::unordered_map<int, std::string> labels_;

    /* ros param */
    int top_k_;
    double score_threshold_;
    bool keep_aspect_ratio_;
    bool image_view_;
    bool verbose_;

    //tf2::Matrix3x3 camera_K_inv_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg){}

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };

} //namespace edgetpu_rospp

