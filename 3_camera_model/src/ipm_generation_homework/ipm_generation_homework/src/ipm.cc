#include "ipm.h"


// 从内参文件（intrinsicFile）和外参文件（extrinsicFile）加载参数，初始化内参矩阵、畸变参数和外参变换。接受两个文件路径（通常为 YAML 或 XML 格式），用于读取相机内参（K_, D_, xi_）和外参（T_vehicle_cam_）。
MEICamera::MEICamera(const std::string& intrinsicFile,
                     const std::string& extrinsicFile) {
  K_ = cv::Mat::eye(3, 3, CV_64F); // 创建 3x3 单位矩阵（双精度浮点型），作为内参矩阵的初始值
  D_ = cv::Mat::zeros(4, 1, CV_64F); // 创建 4x1 零向量，用于存储畸变系数（k1, k2, p1, p2）

  {  // load intrinsic
    cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ); // 使用 OpenCV 的 cv::FileStorage 类以只读模式打开 intrinsicFile（通常为 YAML 文件），用于读取内参和畸变参数
    if (!fs.isOpened()) {
      std::cerr << "Fail to open intrinsic file: " << intrinsicFile
                << std::endl;
      return; // 如果文件无法打开，输出错误信息到标准错误流并退出构造函数，保持 is_valid_ = false
    }
    cv::FileNode m_param = fs["mirror_parameters"];
    cv::FileNode D_param = fs["distortion_parameters"];
    cv::FileNode K_param = fs["projection_parameters"];
    if (m_param.empty() || D_param.empty() || K_param.empty()) {
      std::cerr << "Error intrinsic file: " << intrinsicFile << std::endl;
      return;
    }
    fs["image_width"] >> width_;
    fs["image_height"] >> height_;
    m_param["xi"] >> xi_;
    K_param["gamma1"] >> K_.at<double>(0, 0);
    K_param["gamma2"] >> K_.at<double>(1, 1);
    K_param["u0"] >> K_.at<double>(0, 2);
    K_param["v0"] >> K_.at<double>(1, 2);
    D_param["k1"] >> D_.at<double>(0, 0);
    D_param["k2"] >> D_.at<double>(1, 0);
    D_param["p1"] >> D_.at<double>(2, 0);
    D_param["p2"] >> D_.at<double>(3, 0);
  }

  {  // load extrinsic
    cv::FileStorage fs(extrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open extrinsic file: " << extrinsicFile
                << std::endl;
      return;
    }
    cv::FileNode trans_param = fs["transform"]["translation"];
    cv::FileNode rot_param = fs["transform"]["rotation"];
    if (trans_param.empty() || rot_param.empty()) {
      std::cerr << "Error extrinsic file: " << extrinsicFile << std::endl;
      return;
    }
    fs["frame_id"] >> id_;

    // T_vehicle_cam_ represents the transformation from the camera coordinate system to the vehicle coordinate system,
    // the origin of the vehicle coordinate system is the center of the vehicle.
    trans_param["x"] >> T_vehicle_cam_.translation().x();
    trans_param["y"] >> T_vehicle_cam_.translation().y();
    trans_param["z"] >> T_vehicle_cam_.translation().z();
    Eigen::Quaterniond q;
    rot_param["x"] >> q.x();
    rot_param["y"] >> q.y();
    rot_param["z"] >> q.z();
    rot_param["w"] >> q.w();
    T_vehicle_cam_.linear() = q.toRotationMatrix();
  }

  is_valid_ = true;
}


// 输出相机参数用于调试。打印相机 ID、有效性、分辨率、MEI 模型参数、内参矩阵、畸变系数和外参矩阵，便于验证参数是否正确加载。
void MEICamera::DebugString() const {
  std::cout << id_ << " : " << is_valid_ << std::endl;
  std::cout << "height: " << height_ << std::endl;
  std::cout << "width: " << width_ << std::endl;
  std::cout << "xi: " << xi_ << std::endl;
  std::cout << "K : " << K_ << std::endl;
  std::cout << "D : " << D_ << std::endl;
  std::cout << "cam_extrinsic : " << T_vehicle_cam_.matrix() << std::endl;
}

// IPM 类的默认构造函数。初始化 IPM 对象，默认参数（如 ipm_img_h_ = 1000, pixel_scale_ = 0.02）已在 ipm.h 中定义，无需额外操作。
IPM::IPM() {}

// 添加相机到 IPM 系统。接受内参和外参文件路径，创建 MEICamera 对象并添加到 cameras_ 向量中。允许多相机系统（如自动驾驶中的前后左右相机）。
void IPM::AddCamera(const std::string& intrinsicsFile,
                    const std::string& extrinsicFile) {
  cameras_.emplace_back(intrinsicsFile, extrinsicFile);
  std::cout << "---------ipm add new camera ---------" << std::endl;
  cameras_.back().DebugString(); // 调用新添加相机的 DebugString 函数，显示其参数。
}


// 从多个相机图像（images）生成统一鸟瞰视图，处理畸变和透视变换。输入为 std::vector<cv::Mat>，包含各相机图像；输出为 cv::Mat，表示鸟瞰视图。
cv::Mat IPM::GenerateIPMImage(const std::vector<cv::Mat>& images) const {
  // 初始化鸟瞰图像。创建 1000x1000 的 RGB 图像（CV_8UC3 表示 8 位无符号整数，3 通道），初始为全黑（零值）。
  cv::Mat ipm_image = cv::Mat::zeros(ipm_img_h_, ipm_img_w_, CV_8UC3);

  // 检查输入图像数量是否与相机数量匹配。
  if (images.size() != cameras_.size()) {
    // 如果如果 images 数量与 cameras_ 不一致，输出错误信息并返回初始化的全黑鸟瞰图像。
    std::cout << "IPM not init normaly !" << std::endl;
    return ipm_image;
  }
  
  // 遍历鸟瞰图像的每个像素。对 1000x1000 图像的每个像素 (u, v) 进行处理，计算其对应的地面坐标和相机图像坐标。
  for (int u = 0; u < ipm_img_w_; ++u) {
    for (int v = 0; v < ipm_img_h_; ++v) {
      // 将鸟瞰图像像素 (u, v) 转换为车辆坐标系中的地面点 p_v。
      // Calculate the point p_v in vehicle coordinates, p_v is corresponding to the current pixel (u, v).
      // Assume the height of the ipm_image in vehicle coordinate is 0.
      Eigen::Vector3d p_v(-(0.5 * ipm_img_h_ - u) * pixel_scale_,
                          (0.5 * ipm_img_w_ - v) * pixel_scale_, 0);

      // Iterate over each camera
      for (size_t i = 0; i < cameras_.size(); ++i) {
        // 对每个相机，计算地面点 p_v 在该相机图像平面上的投影
        ////////////////////////TODO begin///////////////////////

        int uv0;
        int uv1;
        ////////////////////////TODO end/////////////////////
        // (uv0, uv1) is the projected pixel from p_v to cameras_[i]
        // 检查投影坐标是否在相机图像范围内。如果 (uv0, uv1)超出相机图像边界（width_, height_），跳过该相机，继续处理下一个相机。
        if (uv0 < 0 || uv0 >= cameras_[i].width_ || uv1 < 0 ||
            uv1 >= cameras_[i].height_) {
          continue;
        }

        // 获取相机图像中对应像素的颜色值，并将其赋值给鸟瞰图像的当前像素。如果鸟瞰图像的当前像素仍为黑色（未填充），直接赋值；否则，将当前像素颜色与新颜色进行平均。
        // 检查鸟瞰图像像素(v, u)是否为黑色（初始值 [0, 0, 0]）。如果是，赋值为相机图像 images[i] 在(uv1, uv0)的 RGB 值（cv::Vec3b 表示 3 通道颜色）。
        // If the IPM image pixel is still black (not yet filled), directly assign the color
        if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
          ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
        } else {
          // 处理重叠区域，平均像素颜色。如果像素已有颜色（非黑色，说明其他相机已贡献），将其与当前相机图像的颜色取平均值，生成融合效果，避免重叠区域突变。
          ipm_image.at<cv::Vec3b>(v, u) = (ipm_image.at<cv::Vec3b>(v, u) +
                                           images[i].at<cv::Vec3b>(uv1, uv0)) /
                                          2;
        }
      }
    }
  }

  // 返回填充完成的 ipm_image，为 1000x1000 的 RGB 图像，表示地面鸟瞰视图。
  return ipm_image;
}