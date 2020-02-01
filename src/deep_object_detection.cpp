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

#include <edgetpu_roscpp/deep_object_detection.h>

namespace edgetpu_roscpp
{
  void DeepObjectDetection::onInit()
  {
    DiagnosticNodelet::onInit();
    /* ros params */
    /** for size filter mode **/
    std::string model_file, model2_file, label_file;
    pnh_->param("model_file", model_file, std::string(""));
    pnh_->param("model2_file", model2_file, std::string(""));
    pnh_->param("label_file", label_file, std::string(""));
    pnh_->param("top_k", top_k_, 2);
    pnh_->param("score_threshold", score_threshold_, 0.3);
    pnh_->param("keep_aspect_ratio", keep_aspect_ratio_, false);
    pnh_->param("verbose", verbose_, false);
    pnh_->param("image_view", image_view_, false);
    always_subscribe_ = true;

    labels_ = coral::ReadLabelFile(label_file);
    detection_engine_ = boost::make_shared<coral::DetectionEngine>(model_file);
    if(model2_file != std::string(""))
      detection2_engine_ = boost::make_shared<coral::DetectionEngine>(model2_file);
    model_tensor_shape_ = detection_engine_->get_input_tensor_shape(); // [1, height, width, 3]
    if(model_tensor_shape_.size() != 4 || model_tensor_shape_.at(0) != 1 || model_tensor_shape_.at(3) != 3)
      throw std::runtime_error("the input tensor shape for classification is not correct");

    if (image_view_) image_pub_ = advertiseImage(*pnh_, "detection_result", 1);

    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);

    onInitPostProcess();
  }

  void DeepObjectDetection::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &DeepObjectDetection::imageCallback, this);
    cam_info_sub_ = nh_->subscribe("camera_info", 1, &DeepObjectDetection::cameraInfoCallback, this);
  }

  void DeepObjectDetection::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void DeepObjectDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat src_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;

    cv::Mat resized_img;
    CvSize2D32f ratio;
    resize(src_img, cv::Size(model_tensor_shape_.at(2), model_tensor_shape_.at(1)), keep_aspect_ratio_, resized_img, ratio);
    std::vector<uint8_t> input_tensor(resized_img.data, resized_img.data + (resized_img.cols * resized_img.rows * resized_img.elemSize()));

    /* first model */
    double start_t = ros::Time::now().toSec();
    auto results = detection_engine_->DetectWithInputTensor(input_tensor, score_threshold_, top_k_);

    if(verbose_) ROS_INFO("deep detection1 result:");

    for (auto result : results)
      {
        result.corners.xmin *= (src_img.size().width / ratio.width);
        result.corners.xmax *= (src_img.size().width / ratio.width);
        result.corners.ymin *= (src_img.size().height / ratio.height);
        result.corners.ymax *= (src_img.size().height / ratio.height);

        if(verbose_)
          {
            std::cout << "---------------------------" << std::endl;
            std::cout << labels_[result.label] << std::endl;
            std::cout << "Score: " << result.score << std::endl;
            std::cout << "Box: ["
                      << result.corners.xmin << ", "
                      << result.corners.ymin << ", "
                      << result.corners.xmax << ", "
                      << result.corners.ymax << "] "
                      << std::endl;
          }

        if(image_view_)
          {
            cv::rectangle(src_img,
                          cv::Point(result.corners.xmin, result.corners.ymin),
                          cv::Point(result.corners.xmax, result.corners.ymax),
                          cv::Scalar(0,0,255), 10);
            cv::putText(src_img, std::to_string(result.score),
                        cv::Point(result.corners.xmin, result.corners.ymin + 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1, cv::Scalar(0,0,255), 2, 8, false);
          }
      }

    double t1 = ros::Time::now().toSec() - start_t;

    if(detection2_engine_ != nullptr)
      {
        start_t = ros::Time::now().toSec();
        results = detection2_engine_->DetectWithInputTensor(input_tensor, score_threshold_, top_k_);

        if(verbose_) ROS_INFO("deep detection2 result:");

        for (auto result : results)
          {
            result.corners.xmin *= (src_img.size().width / ratio.width);
            result.corners.xmax *= (src_img.size().width / ratio.width);
            result.corners.ymin *= (src_img.size().height / ratio.height);
            result.corners.ymax *= (src_img.size().height / ratio.height);

            if(verbose_)
              {
                std::cout << "---------------------------" << std::endl;
                std::cout << labels_[result.label] << std::endl;
                std::cout << "Score: " << result.score << std::endl;
                std::cout << "Box: ["
                          << result.corners.xmin << ", "
                          << result.corners.ymin << ", "
                          << result.corners.xmax << ", "
                          << result.corners.ymax << "] "
                          << std::endl;
              }

            if(image_view_)
              {
                cv::rectangle(src_img,
                              cv::Point(result.corners.xmin, result.corners.ymin),
                              cv::Point(result.corners.xmax, result.corners.ymax),
                              cv::Scalar(255,0,0), 10);
                cv::putText(src_img, std::to_string(result.score),
                            cv::Point(result.corners.xmin, result.corners.ymin + 30),
                            cv::FONT_HERSHEY_SIMPLEX,
                            1, cv::Scalar(255,0,0), 2, 8, false);
              }

          }
        double t2 = ros::Time::now().toSec() - start_t;

        if(verbose_)  ROS_WARN("t1: %f, t2: %f", t1, t2);
      }

    if(image_view_) image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, src_img).toImageMsg());
  }
} //namespace edgetpu_roscpp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (edgetpu_roscpp::DeepObjectDetection, nodelet::Nodelet);
