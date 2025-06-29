#include "ipm.h" 
// 引入头文件 ipm.h，包含了 MEICamera 和 IPM 类的声明

// 构造函数：从内参文件和外参文件加载参数，初始化内参矩阵、畸变参数和外参变换
MEICamera::MEICamera(const std::string& intrinsicFile,
                     const std::string& extrinsicFile) {
  K_ = cv::Mat::eye(3, 3, CV_64F); // 创建3x3单位矩阵，作为内参矩阵初始值
  D_ = cv::Mat::zeros(4, 1, CV_64F); // 创建4x1零向量，作为畸变参数初始值

  {  // 加载内参
    cv::FileStorage fs(intrinsicFile, cv::FileStorage::READ); // 用OpenCV的FileStorage读取内参文件
    if (!fs.isOpened()) {
      std::cerr << "Fail to open intrinsic file: " << intrinsicFile
                << std::endl;
      return; // 打不开文件则报错并返回
    }
    cv::FileNode m_param = fs["mirror_parameters"];
    cv::FileNode D_param = fs["distortion_parameters"];
    cv::FileNode K_param = fs["projection_parameters"];
    if (m_param.empty() || D_param.empty() || K_param.empty()) {
      std::cerr << "Error intrinsic file: " << intrinsicFile << std::endl;
      return; // 缺少关键参数节点则报错并返回
    }
    fs["image_width"] >> width_; // 读取图像宽度
    fs["image_height"] >> height_; // 读取图像高度
    m_param["xi"] >> xi_; // 读取MEI模型的xi参数
    K_param["gamma1"] >> K_.at<double>(0, 0); // 读取内参矩阵的焦距参数
    K_param["gamma2"] >> K_.at<double>(1, 1);
    K_param["u0"] >> K_.at<double>(0, 2); // 读取主点坐标
    K_param["v0"] >> K_.at<double>(1, 2);
    D_param["k1"] >> D_.at<double>(0, 0); // 读取畸变参数
    D_param["k2"] >> D_.at<double>(1, 0);
    D_param["p1"] >> D_.at<double>(2, 0);
    D_param["p2"] >> D_.at<double>(3, 0);
  }

  {  // 加载外参
    cv::FileStorage fs(extrinsicFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "Fail to open extrinsic file: " << extrinsicFile
                << std::endl;
      return; // 打不开文件则报错并返回
    }
    cv::FileNode trans_param = fs["transform"]["translation"];
    cv::FileNode rot_param = fs["transform"]["rotation"];
    if (trans_param.empty() || rot_param.empty()) {
      std::cerr << "Error extrinsic file: " << extrinsicFile << std::endl;
      return; // 缺少关键参数节点则报错并返回
    }
    fs["frame_id"] >> id_; // 读取相机ID

    // T_vehicle_cam_ 表示从相机坐标系到车辆坐标系的变换
    trans_param["x"] >> T_vehicle_cam_.translation().x(); // 读取平移
    trans_param["y"] >> T_vehicle_cam_.translation().y();
    trans_param["z"] >> T_vehicle_cam_.translation().z();
    Eigen::Quaterniond q; // 用四元数表示旋转
    rot_param["x"] >> q.x();
    rot_param["y"] >> q.y();
    rot_param["z"] >> q.z();
    rot_param["w"] >> q.w();
    T_vehicle_cam_.linear() = q.toRotationMatrix(); // 四元数转旋转矩阵
  }

  is_valid_ = true; // 标记相机参数加载成功
}

// 输出相机参数用于调试
void MEICamera::DebugString() const {
  std::cout << id_ << " : " << is_valid_ << std::endl;
  std::cout << "height: " << height_ << std::endl;
  std::cout << "width: " << width_ << std::endl;
  std::cout << "xi: " << xi_ << std::endl;
  std::cout << "K : " << K_ << std::endl;
  std::cout << "D : " << D_ << std::endl;
  std::cout << "cam_extrinsic : " << T_vehicle_cam_.matrix() << std::endl;
}

// IPM类的默认构造函数
IPM::IPM() {}

// 添加相机到IPM系统
void IPM::AddCamera(const std::string& intrinsicsFile,
                    const std::string& extrinsicFile) {
  cameras_.emplace_back(intrinsicsFile, extrinsicFile); // 创建并添加新相机
  std::cout << "---------ipm add new camera ---------" << std::endl;
  cameras_.back().DebugString(); // 输出新相机参数
}

// 生成鸟瞰图（逆透视映射）的核心函数
cv::Mat IPM::GenerateIPMImage(const std::vector<cv::Mat>& images) const {
  cv::Mat ipm_image = cv::Mat::zeros(ipm_img_h_, ipm_img_w_, CV_8UC3); // 创建全黑鸟瞰图

  if (images.size() != cameras_.size()) {
    std::cout << "IPM not init normaly !" << std::endl;
    return ipm_image; // 图像数量和相机数量不一致则报错并返回空图
  }
  
  // 遍历鸟瞰图每个像素
  for (int u = 0; u < ipm_img_w_; ++u) {
    for (int v = 0; v < ipm_img_h_; ++v) {
      // 将鸟瞰图像素(u, v)转换为车辆坐标系下的地面点p_v
      Eigen::Vector3d p_v(-(0.5 * ipm_img_h_ - u) * pixel_scale_,
                          (0.5 * ipm_img_w_ - v) * pixel_scale_, 0);

      // 遍历所有相机
      for (size_t i = 0; i < cameras_.size(); ++i) {
        // 地面点 p_v (车辆坐标系) -> 相机坐标系
        Eigen::Vector3d p_c = cameras_[i].T_vehicle_cam_.inverse() * p_v;

        // MEI模型投影到归一化平面
        // 1. 先将相机坐标归一化
        double X = p_c.x();
        double Y = p_c.y();
        double Z = p_c.z();

        // 2. MEI模型的投影公式
        // 参考公式: https://docs.opencv.org/4.x/dc/dbb/tutorial_camera_calibration.html
        double xi = cameras_[i].xi_;
        double d = sqrt(X*X + Y*Y + Z*Z);
        double denom = Z + xi * d;
        if (denom <= 1e-6) continue; // 防止除零和后方点

        double x_norm = X / denom;
        double y_norm = Y / denom;

        // 3. 加入畸变（此处假设畸变很小可忽略，若需精确可补充畸变校正）
        // 4. 投影到像素坐标
        double u_proj = cameras_[i].K_.at<double>(0,0) * x_norm + cameras_[i].K_.at<double>(0,2);
        double v_proj = cameras_[i].K_.at<double>(1,1) * y_norm + cameras_[i].K_.at<double>(1,2);

        int uv0 = static_cast<int>(std::round(u_proj));
        int uv1 = static_cast<int>(std::round(v_proj));
        // (uv0, uv1) 是地面点p_v在第i个相机图像上的投影像素

        // 检查投影坐标是否在相机图像范围内
        if (uv0 < 0 || uv0 >= cameras_[i].width_ || uv1 < 0 ||
            uv1 >= cameras_[i].height_) {
          continue;
        }

        // 如果鸟瞰图像素还没填色，则直接赋值；否则与已有颜色取平均
        if (ipm_image.at<cv::Vec3b>(v, u) == cv::Vec3b(0, 0, 0)) {
          ipm_image.at<cv::Vec3b>(v, u) = images[i].at<cv::Vec3b>(uv1, uv0);
        } else {
          ipm_image.at<cv::Vec3b>(v, u) = (ipm_image.at<cv::Vec3b>(v, u) +
                                           images[i].at<cv::Vec3b>(uv1, uv0)) /
                                          2;
        }
      }
    }
  }

  return ipm_image; // 返回生成的鸟瞰图
}