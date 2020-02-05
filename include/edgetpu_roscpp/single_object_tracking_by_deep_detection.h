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
#include <jsk_recognition_msgs/BoundingBox.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "src/cpp/detection/engine.h"
#include "src/cpp/examples/label_utils.h"
#include "src/cpp/examples/model_utils.h"
#include <tuple>

#define DETECTING_TARGET cv::Scalar(0,0,255) // blue
#define TRACKING_TARGET  cv::Scalar(0,255,0) // green
#define LOSING_TARGET    cv::Scalar(255,255,0) // yellow
#define EXPANDED_BOUNDING_BOX    cv::Scalar(255,0,0) // red


namespace edgetpu_roscpp
{
  class SingleObjectDeepTrackingDetection: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    SingleObjectDeepTrackingDetection():
      DiagnosticNodelet("DeepObjectDetection"),
      model_tensor_shape_(0),
      detected_(false),
      detected_in_this_frame_(false),
      lost_target_(false),
      detected_frame_cnt_(0),
      detecting_frame_cnt_(0),
      lost_target_frame_cnt_(0),
      status_color_(0),
      image_pub_t_(0)
    {}

    coral::BoxCornerEncoding addOffsetForBoundingBox(int offset_x, int offset_y, const coral::BoxCornerEncoding bounding_box)
    {
      coral::BoxCornerEncoding new_bounding_box = bounding_box;
      new_bounding_box.xmin += offset_x;
      new_bounding_box.xmax += offset_x;
      new_bounding_box.ymin += offset_y;
      new_bounding_box.ymax += offset_y;

      return new_bounding_box;
    }


  protected:
    /* ros publisher */
    ros::Publisher target_bbox_pub_;
    image_transport::Publisher image_pub_;

    /* ros subscriber */
    image_transport::Subscriber image_sub_;

    /* image transport */
    boost::shared_ptr<image_transport::ImageTransport> it_;

    /* detection and tracking */
    boost::shared_ptr<coral::DetectionEngine> detection_engine_;
    std::vector<int> model_tensor_shape_;
    bool detected_in_this_frame_; // a one-shot (one frame) flag
    bool detected_; // a long-term flag
    bool lost_target_;
    int detecting_frame_cnt_; // the total frame count in detection phase
    int detected_frame_cnt_; // the amount of the frame that  continuously detects the a candidate
    int lost_target_frame_cnt_;
    cv::Scalar status_color_;
    coral::BoxCornerEncoding expanded_bounding_box_;
    coral::BoxCornerEncoding prev_expanded_bounding_box_;
    coral::DetectionCandidate best_detection_candidate_;
    coral::DetectionCandidate prev_best_detection_candidate_;

    /* ros param */
    double coarse_detection_score_threshold_;
    double refined_detection_score_threshold_;
    double tracking_score_threshold_;
    double expanding_bounding_box_rate_;
    double expanding_bounding_box_aspect_ratio_;
    double larger_expanding_bounding_box_rate_;
    double image_pub_throttle_rate_;
    double image_pub_t_;
    int detection_check_frame_num_;
    int lost_target_check_frame_num_;
    int redetection_after_lost_target_frame_num_;
    bool quick_detection_;
    bool keep_aspect_ratio_in_inference_;
    bool image_view_;
    bool verbose_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    virtual void detection_tracking_process(cv::Mat& src_img);
    virtual void publish(const std_msgs::Header& msg_header, const cv::Mat& src_img);

    void expandedBoundingImage(const cv::Mat& src_img, const coral::BoxCornerEncoding bounding_box, double expanding_bounding_box_rate,
                               cv::Mat& dst_img, coral::BoxCornerEncoding& expanded_bounding_box);

    std::vector<coral::DetectionCandidate> deepDetectionCore(boost::shared_ptr<coral::DetectionEngine> detection_engine, const cv::Mat input_img, double score_threshold, int candidate_num = 1);

  };

} //namespace edgetpu_rospp

