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

#include <edgetpu_roscpp/single_object_tracking_by_deep_detection.h>

namespace edgetpu_roscpp
{
  void SingleObjectDeepTrackingDetection::onInit()
  {
    DiagnosticNodelet::onInit();
    /* ros params */
    /** for size filter mode **/
    std::string model_file, label_file;
    pnh_->param("model_file", model_file, std::string(""));
    //pnh_->param("detection_candidate_num_per_frame", detection_candidate_num_per_frame_, 1); // this may not need
    pnh_->param("coarse_detection_score_threshold", coarse_detection_score_threshold_, 0.3);
    pnh_->param("refined_detection_score_threshold", refined_detection_score_threshold_, 0.8);
    pnh_->param("tracking_score_threshold", tracking_score_threshold_, 0.6);
    pnh_->param("expanding_bounding_box_rate", expanding_bounding_box_rate_, 2.0);
    pnh_->param("expanding_bounding_box_aspect_ratio", expanding_bounding_box_aspect_ratio_, -1.0);
    pnh_->param("larger_expanding_bounding_box_rate", larger_expanding_bounding_box_rate_, 2.0);
    pnh_->param("target_quality_check_duration", target_quality_check_duration_, 5.0); // second
    pnh_->param("detection_check_frame_num", detection_check_frame_num_, 10);
    pnh_->param("lost_target_check_frame_num", lost_target_check_frame_num_, 5);
    pnh_->param("redetection_after_lost_target_frame_num", redetection_after_lost_target_frame_num_, 5);

    pnh_->param("quick_detection", quick_detection_, false);
    pnh_->param("keep_aspect_ratio_in_inference", keep_aspect_ratio_in_inference_, false);
    pnh_->param("verbose", verbose_, false);
    pnh_->param("image_view", image_view_, false);

    //always_subscribe_ = true;

    detection_engine_ = boost::make_shared<coral::DetectionEngine>(model_file);
    model_tensor_shape_ = detection_engine_->get_input_tensor_shape(); // [1, height, width, 3]
    if(model_tensor_shape_.size() != 4 || model_tensor_shape_.at(0) != 1 || model_tensor_shape_.at(3) != 3)
      throw std::runtime_error("the input tensor shape for classification is not correct");

    if (image_view_) image_pub_ = advertiseImage(*pnh_, "detection_result_image", 1);

    target_bbox_pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*nh_, "target_object_bounding_box", 1);
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);

    onInitPostProcess();
  }

  void SingleObjectDeepTrackingDetection::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &SingleObjectDeepTrackingDetection::imageCallback, this);
    cam_info_sub_ = nh_->subscribe("camera_info", 1, &SingleObjectDeepTrackingDetection::cameraInfoCallback, this);
  }

  void SingleObjectDeepTrackingDetection::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void SingleObjectDeepTrackingDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat src_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
    detection_tracking_process(msg->header, src_img);

    if(image_view_) publishImg(msg->header, src_img);
  }

  void SingleObjectDeepTrackingDetection::publishImg(const std_msgs::Header& msg_header, const cv::Mat& src_img)
  {
    if(detected_in_this_frame_)
      image_pub_.publish(cv_bridge::CvImage(msg_header, sensor_msgs::image_encodings::RGB8, src_img).toImageMsg());
    else
      image_pub_.publish(cv_bridge::CvImage(msg_header, sensor_msgs::image_encodings::BGR8, src_img).toImageMsg());
  }

  std::vector<coral::DetectionCandidate> SingleObjectDeepTrackingDetection::deepDetectionCore(const cv::Mat input_img, double score_threshold)
  {
    cv::Mat resized_img;
    CvSize2D32f ratio(1.0, 1.0);
    resize(input_img, cv::Size(model_tensor_shape_.at(2), model_tensor_shape_.at(1)), keep_aspect_ratio_in_inference_, resized_img, ratio);
    std::vector<uint8_t> input_tensor(resized_img.data, resized_img.data + (resized_img.cols * resized_img.rows * resized_img.elemSize()));

    auto results = detection_engine_->DetectWithInputTensor(input_tensor, score_threshold, 1);

    for (auto& result: results)
      {
        result.corners.xmin *= (input_img.size().width / ratio.width);
        result.corners.xmax *= (input_img.size().width / ratio.width);
        result.corners.ymin *= (input_img.size().height / ratio.height);
        result.corners.ymax *= (input_img.size().height / ratio.height);
      }

    return results;
  };


  void SingleObjectDeepTrackingDetection::expandedBoundingImage(const cv::Mat& src_img, const coral::BoxCornerEncoding bounding_box, double expanding_bounding_box_rate, cv::Mat& dst_img, coral::BoxCornerEncoding& expanded_bounding_box)
  {
    int bounding_box_width = bounding_box.xmax - bounding_box.xmin;
    int bounding_box_height = bounding_box.ymax - bounding_box.ymin;
    int bounding_box_center_x = (bounding_box.xmax + bounding_box.xmin) / 2;
    int bounding_box_center_y = (bounding_box.ymax + bounding_box.ymin) / 2;

    int expanded_width = bounding_box_width * expanding_bounding_box_rate;
    int expanded_height = bounding_box_height * expanding_bounding_box_rate;

    if(expanding_bounding_box_aspect_ratio_ > 0)
      {
        if(expanded_width < expanded_height * expanding_bounding_box_aspect_ratio_) expanded_width = expanded_height * expanding_bounding_box_aspect_ratio_;
        else expanded_height = expanded_width / expanding_bounding_box_aspect_ratio_;
      }

    //std::cout << "bounding_box :[" << bounding_box_width << ", " << bounding_box_height << "]" << std::endl;
    //std::cout << "expanded bounding_box :[" << expanded_width << ", " << expanded_height << "], ratio is " << (double)expanded_width / expanded_height << std::endl;

    expanded_bounding_box.xmin = bounding_box_center_x - expanded_width / 2;
    expanded_bounding_box.ymin = bounding_box_center_y - expanded_height / 2;
    expanded_bounding_box.xmax = bounding_box_center_x + expanded_width / 2;
    expanded_bounding_box.ymax = bounding_box_center_y + expanded_height / 2;

    if (expanded_bounding_box.xmin < 0)
      {
        expanded_bounding_box.xmin = 0;
        expanded_bounding_box.xmax = expanded_width;
      }
    if (expanded_bounding_box.xmax >= src_img.size().width)
      {
        expanded_bounding_box.xmax = src_img.size().width - 1;
        expanded_bounding_box.xmin = expanded_bounding_box.xmax - expanded_width;
      }
    if (expanded_bounding_box.ymin < 0)
      {
        expanded_bounding_box.ymin = 0;
        expanded_bounding_box.ymax = expanded_height;
      }
    if (expanded_bounding_box.ymax >= src_img.size().height)
      {
        expanded_bounding_box.ymax = src_img.size().height - 1;
        expanded_bounding_box.ymin = expanded_bounding_box.ymax - expanded_height;
      }

    if (expanded_bounding_box.xmin < 0 || expanded_bounding_box.xmax >= src_img.size().width)
      {
        expanded_bounding_box.xmin = 0;
        expanded_bounding_box.xmax = src_img.size().width -1;
      }

    if (expanded_bounding_box.ymin < 0 || expanded_bounding_box.ymax >= src_img.size().height)
      {
        expanded_bounding_box.ymin = 0;
        expanded_bounding_box.ymax = src_img.size().height -1;
      }

    //printf("expanded bound box: [%f, %f, %f, %f] \n", expanded_bounding_box.xmin, expanded_bounding_box.ymin, expanded_bounding_box.xmax,expanded_bounding_box.ymax);

    dst_img = src_img(cv::Rect(expanded_bounding_box.xmin, expanded_bounding_box.ymin, expanded_bounding_box.xmax - expanded_bounding_box.xmin, expanded_bounding_box.ymax - expanded_bounding_box.ymin));
  };

  void SingleObjectDeepTrackingDetection::detection_tracking_process(const std_msgs::Header& msg_header, cv::Mat& src_img)
  {
    double start_t = ros::Time::now().toSec();

    coral::DetectionCandidate best_detection_candidate;
    best_detection_candidate.score = 0;
    best_detection_candidate.corners.xmin = 0;
    best_detection_candidate.corners.ymin = 0;
    best_detection_candidate.corners.xmax = 0;
    best_detection_candidate.corners.ymax = 0;
    coral::BoxCornerEncoding expanded_bounding_box;
    cv::Mat expanded_bounding_img;

    // ROS_ERROR("detected_frame_cnt_: %d", detected_frame_cnt_);

    /* do detection if necessary */
    if(!detected_)
      {
        detected_in_this_frame_ = false;

        std::vector<coral::DetectionCandidate> detection_candidates;

        auto coarseToRefinedDetection = [&, this](int xmin, int ymin, int width, int height)
        {
          /* for quick detection */
          /* so we only recognize the first candidate that has hight score than refined_detection_score_threshold */
          if(detected_in_this_frame_  && quick_detection_) return;

          auto dst_img = src_img(cv::Rect(xmin, ymin, width, height));
          detection_candidates = deepDetectionCore(dst_img, coarse_detection_score_threshold_);

          if(verbose_) ROS_INFO("coarse detection founds %d candidate in [%d, %d, %d, %d]",
                                (int)detection_candidates.size(), xmin, ymin, xmin + width, ymin + height);

          if(detection_candidates.size() > 0)
            {
              detection_candidates.at(0).corners = addOffsetForBoundingBox(xmin, ymin, detection_candidates.at(0).corners);

              if(verbose_) ROS_INFO("Found coarse candidate in [%d, %d, %d, %d]. Score: %f, BB: [%f, %f, %f, %f]",
                                    xmin, ymin, xmin + width, ymin + height,
                                    detection_candidates.at(0).score,
                                    detection_candidates.at(0).corners.xmin, detection_candidates.at(0).corners.ymin,
                                    detection_candidates.at(0).corners.xmax, detection_candidates.at(0).corners.ymax);


              expandedBoundingImage(src_img, detection_candidates.at(0).corners, expanding_bounding_box_rate_, expanded_bounding_img, expanded_bounding_box);

#if 0
              /* quick detection */
              /* corner case: coarse detection provides a score higher than refined_detection_score_threshold,
                 but the refined detection provides a worse result, even a no-detection
                 so, we suggest to to coarse -> refined detection, all though this will take twice time to do detection.
              */
              if(detection_candidates.at(0).score > refined_detection_score_threshold_ && quick_detection_)
                {
                  best_detection_candidate = detection_candidates.at(0);
                  detected_in_this_frame_ = true;
                  return ;
                }
#endif
              /* refined detection */
              detection_candidates = deepDetectionCore(expanded_bounding_img, refined_detection_score_threshold_);

              if(detection_candidates.size() > 0)
                {
                  if(detection_candidates.at(0).score >= best_detection_candidate.score)
                    {
                      best_detection_candidate = detection_candidates.at(0);
                      detected_in_this_frame_ = true;
                      best_detection_candidate.corners = addOffsetForBoundingBox(expanded_bounding_box.xmin,
                                                                                 expanded_bounding_box.ymin,
                                                                                 best_detection_candidate.corners);

                      if(verbose_) ROS_INFO("Found refined candidate in [%d, %d, %d, %d]. Score: %f, BB: [%f, %f, %f, %f]",
                                            xmin, ymin, xmin + width, ymin + height,
                                            best_detection_candidate.score,
                                            best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin,
                                            best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax);
                    }
                }
              else
                {
                  if(verbose_) ROS_WARN("weak detection result in refined detection mode");
                }

            }
        };

        /* coarse detection with original image */
        if(verbose_) ROS_INFO("coarse detection start in full size"); // debug
        coarseToRefinedDetection(0, 0, src_img.size().width, src_img.size().height);

        /* do five small section detection for the orignal image */
        if(verbose_) ROS_INFO("Start five section detection method");

        /* top-left, down-left, top-right, down-right */
        for (int i = 0; i < 2; i++)
          {
            for (int j = 0; j < 2; j++)
              {
                coarseToRefinedDetection(i * src_img.size().width / 2, j * src_img.size().height / 2, src_img.size().width / 2, src_img.size().height / 2);
              }
          }
        /* center */
        coarseToRefinedDetection(src_img.size().width / 4, src_img.size().height / 4, src_img.size().width / 2, src_img.size().height / 2);

        if(detected_in_this_frame_)
          {
            if(detected_frame_cnt_ == 0) detected_frame_cnt_++;
            else
              {
                /* check whether the target moves continuously */
                if(best_detection_candidate.corners.xmin >= prev_expanded_bounding_box_.xmin &&
                   best_detection_candidate.corners.ymin >= prev_expanded_bounding_box_.ymin &&
                   best_detection_candidate.corners.xmax <= prev_expanded_bounding_box_.xmax &&
                   best_detection_candidate.corners.ymin <= prev_expanded_bounding_box_.ymax)
                  {
                    detected_frame_cnt_++;
                  }
              }

            int detection_check_frame_num = detection_check_frame_num_;
            if (lost_target_) detection_check_frame_num /= 2;

            if(detected_frame_cnt_ == detection_check_frame_num)
              {
                detected_ = true;
                lost_target_ = false;

                ROS_WARN("Detected target!. Score: %f, BB: [%f, %f, %f, %f]",
                         best_detection_candidate.score,
                         best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin,
                         best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax);

                detecting_frame_cnt_ = 0;
                lost_target_frame_cnt_ = lost_target_check_frame_num_; // reset
              }

            prev_expanded_bounding_box_ = expanded_bounding_box;
            prev_best_detection_candidate_ = best_detection_candidate;
          }
        else
          {
            if(verbose_) ROS_WARN("not found target candidate");
            detected_frame_cnt_ = 0;
          }

        if (detected_in_this_frame_)
          {
            cv::rectangle(src_img,
                          cv::Point(best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin),
                          cv::Point(best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax),
                          DETECTING_TARGET, 10);
            cv::putText(src_img, std::to_string(best_detection_candidate.score),
                        cv::Point(best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin + 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1, DETECTING_TARGET, 2, 8, false);

          }

        //ROS_WARN("score: %f in [%f, %f, %f, %f]", best_detection_candidate.score, best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin, best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax);

        detecting_frame_cnt_ ++;

        if(lost_target_ && detecting_frame_cnt_ > redetection_after_lost_target_frame_num_)
          {
            ROS_WARN("Total lost target, so we back the normal detection process (i.e., no quick redtection in tracking mode)");
            lost_target_ = false;
          }
      }

    /* do tracking */
    if(detected_)
      {
        detected_in_this_frame_ = true;

        bool larger_expanding_bbox = false;
        cv::Scalar status_color = TRACKING_TARGET;

        expandedBoundingImage(src_img, prev_best_detection_candidate_.corners, expanding_bounding_box_rate_, expanded_bounding_img, expanded_bounding_box);

        auto detection_candidates = deepDetectionCore(expanded_bounding_img, tracking_score_threshold_);

        if(detection_candidates.size() > 0)
          {
            lost_target_frame_cnt_ = lost_target_check_frame_num_; // reset
            best_detection_candidate = detection_candidates.at(0);
            best_detection_candidate.corners = addOffsetForBoundingBox(expanded_bounding_box.xmin,
                                                                       expanded_bounding_box.ymin,
                                                                       best_detection_candidate.corners);
            if(verbose_)
              {
                ROS_INFO("Found taget candidate in tracking mode. Score: %f, BB: [%f, %f, %f, %f], Expanded BB: [%f, %f, %f, %f]",
                         best_detection_candidate.score,
                         best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin,
                         best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax,
                         expanded_bounding_box.xmin, expanded_bounding_box.ymin,
                         expanded_bounding_box.xmax, expanded_bounding_box.ymax);
              }
          }
        else
          {
            /* re-detection by using broader expanded_bouding_box */
            expandedBoundingImage(src_img, prev_best_detection_candidate_.corners, larger_expanding_bounding_box_rate_, expanded_bounding_img, expanded_bounding_box);
            auto detection_candidates = deepDetectionCore(expanded_bounding_img, tracking_score_threshold_);

            if(detection_candidates.size() > 0)
              {
                lost_target_frame_cnt_ = lost_target_check_frame_num_; // reset
                best_detection_candidate = detection_candidates.at(0);
                best_detection_candidate.corners = addOffsetForBoundingBox(expanded_bounding_box.xmin,
                                                                           expanded_bounding_box.ymin,
                                                                           best_detection_candidate.corners);
                if(verbose_)
                  {
                    ROS_INFO("Found taget candidate in tracking mode (redetection). Score: %f, BB: [%f, %f, %f, %f], Expanded BB: [%f, %f, %f, %f]",
                             best_detection_candidate.score,
                             best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin,
                             best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax,
                             expanded_bounding_box.xmin, expanded_bounding_box.ymin,
                             expanded_bounding_box.xmax, expanded_bounding_box.ymax);
                  }
              }
            else
              {
                lost_target_frame_cnt_--;

                best_detection_candidate = prev_best_detection_candidate_; // keep the last result

                if(verbose_) ROS_INFO("Losing target after redetection.");

                status_color = LOSING_TARGET;
                if(lost_target_frame_cnt_ <= 0)
                  {
                    ROS_WARN_STREAM("Lost target with tracking score threshould " << tracking_score_threshold_);
                    lost_target_ = true;
                    detected_ = false;
                    detected_in_this_frame_ = false;
                    detected_frame_cnt_ = 0;
                  }
                //target_quality_check_timestamp_ =  0;
              }
            larger_expanding_bbox = true;
          }

        /* do target quality check */
        if(ros::Time::now().toSec() - target_quality_check_timestamp_ > target_quality_check_duration_)
          {
            target_quality_check_timestamp_ = ros::Time::now().toSec();
            // TODO
          }

        cv::Scalar expanded_bounding_box_color(255,0,0);
        if(larger_expanding_bbox) expanded_bounding_box_color = cv::Scalar(255,165,0);
        cv::rectangle(src_img,
                      cv::Point(expanded_bounding_box.xmin, expanded_bounding_box.ymin),
                      cv::Point(expanded_bounding_box.xmax, expanded_bounding_box.ymax),
                      expanded_bounding_box_color, 3);

        if (detected_in_this_frame_)
          {
            /*
              cv::rectangle(src_img,
              cv::Point(prev_best_detection_candidate_.corners.xmin, prev_best_detection_candidate_.corners.ymin),
              cv::Point(prev_best_detection_candidate_.corners.xmax, prev_best_detection_candidate_.corners.ymax),
              cv::Scalar(255,0,255), 2);
            */
            cv::putText(src_img, std::to_string(best_detection_candidate.score),
                        cv::Point(best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin + 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1, status_color, 2, 8, false);

            cv::rectangle(src_img,
                          cv::Point(best_detection_candidate.corners.xmin, best_detection_candidate.corners.ymin),
                          cv::Point(best_detection_candidate.corners.xmax, best_detection_candidate.corners.ymax),
                          status_color, 10);
          }


        /* update */
        prev_expanded_bounding_box_ = expanded_bounding_box;
        prev_best_detection_candidate_ = best_detection_candidate;
      }

    /* publish */
    jsk_recognition_msgs::BoundingBox target_bouding_box_msg;
    target_bouding_box_msg.header = msg_header;
    target_bouding_box_msg.value = detected_?best_detection_candidate.score:-1;
    target_bouding_box_msg.label = best_detection_candidate.label;
    target_bouding_box_msg.pose.position.x = best_detection_candidate.corners.xmin;
    target_bouding_box_msg.pose.position.y = best_detection_candidate.corners.ymin;
    target_bouding_box_msg.dimensions.x = best_detection_candidate.corners.xmax - best_detection_candidate.corners.xmin;
    target_bouding_box_msg.dimensions.y = best_detection_candidate.corners.ymax - best_detection_candidate.corners.ymin;
    target_bbox_pub_.publish(target_bouding_box_msg);

    if(verbose_) ROS_INFO_STREAM("............... Total process time for this frame is " << ros::Time::now().toSec() - start_t);
  }
} //namespace edgetpu_roscpp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (edgetpu_roscpp::SingleObjectDeepTrackingDetection, nodelet::Nodelet);
