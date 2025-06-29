#include "ipm.h"

int main(int argc, char *argv[]) {
  IPM ipm;

  constexpr int CAM_NUM = 4;
  std::vector<cv::Mat> raw_images(CAM_NUM);
  for (int i = 0; i < CAM_NUM; ++i) {
    std::string ex_file(IMAGE_DIR + std::to_string(i) + "_extrinsic.yaml");
    std::string in_file(IMAGE_DIR + std::to_string(i) + "_intrinsic.yaml");

    ipm.AddCamera(in_file, ex_file);
    raw_images[i] = cv::imread(IMAGE_DIR + std::to_string(i) + ".png");
  }

  cv::Mat ipm_img = ipm.GenerateIPMImage(raw_images);
  cv::imwrite(IMAGE_DIR "/ipm_img.png", ipm_img);
  std::cout << "Result saved in: " IMAGE_DIR "/ipm_img.png" << std::endl;

  cv::namedWindow("ipm_img", cv::WINDOW_NORMAL);
  cv::imshow("ipm_img", ipm_img);
  cv::waitKey();
  return 0;
}