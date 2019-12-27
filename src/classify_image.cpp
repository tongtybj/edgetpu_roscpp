// An example to classify image.
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/cpp/classification/engine.h"
#include "src/cpp/examples/label_utils.h"
#include "src/cpp/examples/model_utils.h"
#include <opencv2/opencv.hpp>
#include "edgetpu_roscpp/image_resize.h"
#include <chrono>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

ABSL_FLAG(std::string, model_path,
          "/tmp/mobilenet_v1_1.0_224_quant_edgetpu.tflite",
          "Path to the tflite model.");

ABSL_FLAG(std::string, image_path, "/tmp/cat.bmp",
          "Path to the image to be classified.");

ABSL_FLAG(std::string, labels_path, "/tmp/imagenet_labels.txt",
          "Path to the imagenet labels.");

void ClassifyImage(const std::string& model_path, const std::string& image_path,
                   const std::string& labels_path) {
  // Load the model.
  coral::ClassificationEngine engine(model_path);
  std::vector<int> input_tensor_shape = engine.get_input_tensor_shape(); // [1, height, width, 3]
  if(input_tensor_shape.size() != 4 || input_tensor_shape.at(0) != 1 || input_tensor_shape.at(3) != 3)
    throw std::runtime_error("the input tensor shape for classification is not correct");

  // Read the image.
  auto t0 = Time::now();
  std::cout << "image is:" << image_path << std::endl;
  cv::Mat input_img = cv::imread(image_path);
  cv::cvtColor(input_img, input_img, CV_BGR2RGB);

  cv::Mat output_img;
  CvSize2D32f ratio;
  resize(input_img, cv::Size(input_tensor_shape.at(2), input_tensor_shape.at(1)), false, output_img, ratio);
  std::vector<uint8_t> input_tensor(output_img.data, output_img.data + (output_img.cols * output_img.rows * output_img.elemSize()));
  auto t1 = Time::now();
  fsec fs = t1 - t0;
  ms d = std::chrono::duration_cast<ms>(fs);
  //std::cout << fs.count() << "s\n";

  // Read the label file.
  auto labels = coral::ReadLabelFile(labels_path);

  auto results = engine.ClassifyWithInputTensor(input_tensor);
  for (auto result : results) {
    std::cout << "---------------------------" << std::endl;
    std::cout << labels[result.id] << std::endl;
    std::cout << "Score: " << result.score << std::endl;
  }
}

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  ClassifyImage(absl::GetFlag(FLAGS_model_path),
                absl::GetFlag(FLAGS_image_path),
                absl::GetFlag(FLAGS_labels_path));
}
