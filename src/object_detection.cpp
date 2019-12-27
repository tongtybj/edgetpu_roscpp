// An example to object detection.
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/cpp/detection/engine.h"
#include "src/cpp/examples/label_utils.h"
#include "src/cpp/examples/model_utils.h"
#include <opencv2/opencv.hpp>
#include "edgetpu_roscpp/image_resize.h"
#include <chrono>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

ABSL_FLAG(std::string, model_path,
          "/tmp/ssd_mobilenet_v1_fine_tuned_edgetpu.tflite",
          "Path to the tflite model.");

ABSL_FLAG(std::string, image_path, "/tmp/pets.jpg",
          "Path to the image to be classified.");

ABSL_FLAG(std::string, labels_path, "/tmp/pet_labels.txt",
          "Path to the imagenet labels.");

ABSL_FLAG(bool, keep_aspect_ratio, false,
          "keep the image aspect ratio when down-sampling the image by adding black pixel padding (zeros) on bottom or right. By default the image is resized and reshaped without cropping. This option should be the same as what is applied on input images during model training. Otherwise the accuracy may be affected and the  bounding box of detection result may be stretched.");

void ObjectDetection(const std::string& model_path, const std::string& image_path,
                     const std::string& labels_path, const bool& keep_aspect_ratio)
{
  // Load the model.
  coral::DetectionEngine engine(model_path);
  std::vector<int> input_tensor_shape = engine.get_input_tensor_shape(); // [1, height, width, 3]
  if(input_tensor_shape.size() != 4 || input_tensor_shape.at(0) != 1 || input_tensor_shape.at(3) != 3)
    {
      std::cout << "input tnesor shape: [";
      for (auto element: input_tensor_shape)
        std::cout << element << ", ";
      std::cout <<"]" << std::endl;

      throw std::runtime_error("the input tensor shape for classification is not correct");
    }

  // Read the image.
  cv::Mat raw_img = cv::imread(image_path);
  cv::Mat input_img;
  cv::cvtColor(raw_img, input_img, CV_BGR2RGB);
  auto t0 = Time::now();
  //std::cout << "image: [" << input_img.size().width << ", "<< input_img.size().height << "]" << std::endl;
  auto t1 = Time::now();
  fsec fs = t1 - t0;
  //std::cout << "color convert: " << fs.count() << "s\n";

  cv::Mat output_img;
  CvSize2D32f ratio;

  t0 = Time::now();
  resize(input_img, cv::Size(input_tensor_shape.at(2), input_tensor_shape.at(1)), keep_aspect_ratio, output_img, ratio);
  std::vector<uint8_t> input_tensor(output_img.data, output_img.data + (output_img.cols * output_img.rows * output_img.elemSize()));
  t1 = Time::now();
  fs = t1 - t0;
  //std::cout << "resize: " << fs.count() << "s\n";
  //std::cout << "ratio: [" << ratio.width << ", " <<  ratio.height << "]" << std::endl;

  // Read the label file.
  auto labels = coral::ReadLabelFile(labels_path);

  auto results = engine.DetectWithInputTensor(input_tensor);
  for (auto result : results) {
    std::cout << "---------------------------" << std::endl;
    std::cout << labels[result.label] << std::endl;
    std::cout << "Score: " << result.score << std::endl;
    result.corners.xmin *= (input_img.size().width / ratio.width);
    result.corners.xmax *= (input_img.size().width / ratio.width);
    result.corners.ymin *= (input_img.size().height / ratio.height);
    result.corners.ymax *= (input_img.size().height / ratio.height);

    std::cout << "Box: ["
              << result.corners.xmin << ", "
              << result.corners.ymin << ", "
              << result.corners.xmax << ", "
              << result.corners.ymax << "] "
              << std::endl;

    cv::rectangle(raw_img,
                  cv::Point(result.corners.xmin, result.corners.ymin),
                  cv::Point(result.corners.xmax, result.corners.ymax),
                  cv::Scalar(0,0,255), 10);
    cv::putText(raw_img, std::to_string(result.score),
                cv::Point(result.corners.xmin, result.corners.ymin + 30),
                cv::FONT_HERSHEY_SIMPLEX,
                1, cv::Scalar(0,0,255), 2, 8, false);
  }

  cv::imwrite("/tmp/object_detection.png", raw_img);
  std::cout << "Please check /tmp/object_detection.png" << std::endl;
}

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  ObjectDetection(absl::GetFlag(FLAGS_model_path),
                  absl::GetFlag(FLAGS_image_path),
                  absl::GetFlag(FLAGS_labels_path),
                  absl::GetFlag(FLAGS_keep_aspect_ratio));
}
