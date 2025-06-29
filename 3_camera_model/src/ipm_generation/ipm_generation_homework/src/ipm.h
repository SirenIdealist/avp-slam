#pragma once // 确保编译器只包含该头文件一次，避免重复定义导致的编译错误。相比传统的头文件保护宏（如 #ifndef），它更简洁且不易出错
#include <eigen3/Eigen/Eigen> // Eigen 库的主头文件，包含所有核心功能
#include <eigen3/Eigen/Core> // 提供矩阵、向量等基本数据结构和运算
#include <eigen3/Eigen/Geometry> // 提供几何变换相关功能，如旋转矩阵、平移向量和仿射变换
#include <iostream>
// #include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp> // opencv2/opencv.hpp 包含 OpenCV 的核心功能（如 cv::Mat）、图像处理、相机标定等，用于处理图像数据和实现 IPM 变换
#include <string> // 提供 std::string 类，用于存储相机 ID 或文件路径
#include <vector> // 提供 std::vector 容器，用于存储多个相机对象或图像数据


// MEICamera 主要是用来存储相机参数（内参、外参、畸变等），本质上更像一个“数据结构体”，而不是复杂的面向对象实体。C++ 推荐这种场景用 struct
struct MEICamera { // 定义 MEICamera 结构体，用于封装相机参数, 表示一个相机，包含内参、外参和畸变参数
  MEICamera(const std::string &intrinsicFile, const std::string &extrinsicFile); // intrinsicFile：包含相机内参和畸变参数的文件路径; extrinsicFile：包含相机外参（相对于车辆坐标系的变换）的文件路径。 构造函数负责从文件中读取参数并初始化相机
  ~MEICamera() = default; // 析构函数, 使用 = default 表示采用编译器默认的析构行为。由于没有动态分配的资源，析构函数无需额外实现。

  void DebugString() const; // 声明调试函数，用于输出相机参数（如内参、外参、ID）到控制台，便于调试。const 保证函数不修改对象。

  bool is_valid_{false}; // 标志相机是否有效，用于检查相机参数是否正确加载
  std::string id_; // 存储相机标识，用于区分多相机系统中的不同相机
  int height_, width_; // 存储相机图像的尺寸
  double xi_; // 存储 MEI 相机模型的畸变参数
  cv::Mat K_, D_; // 存储相机内参和畸变参数
  Eigen::Affine3d T_vehicle_cam_; // 存储相机相对于车辆的外参, T_vehicle_cam_ 是一个 4x4 仿射变换矩阵（Eigen::Affine3d），表示从相机坐标系到车辆坐标系的变换，包含 旋转 和 平移 , 用于将相机坐标系中的点转换到车辆坐标系，IPM 依赖此参数进行地面映射。
};

class IPM { // 定义 IPM 类，用于实现逆透视映射。IPM 类管理多个相机，处理输入图像并生成鸟瞰视图。public 部分包含对外接口
 public:
  IPM();
  ~IPM() = default;


  // 添加相机到 IPM 系统, 函数接受内参和外参文件路径，创建 MEICamera 对象并添加到 cameras_ 向量中。允许多相机系统（如自动驾驶中的前后左右相机）。
  void AddCamera(const std::string &intrinsicsFile,
                 const std::string &extrinsicFile);


  // GenerateIPMImage 是核心功能，处理输入图像，应用畸变校正和逆透视映射，生成统一鸟瞰视图
  // 输入：images 是一个 std::vector<cv::Mat>，包含多个相机的输入图像（cv::Mat 是 OpenCV 的图像矩阵）。
  // 输出：返回一个 cv::Mat，表示生成的鸟瞰视图图像。
  // 功能：基于相机参数（K_, D_, T_vehicle_cam_），对输入图像应用逆透视映射，可能包括畸变校正和单应性变换，合并多相机视图。
  cv::Mat GenerateIPMImage(const std::vector<cv::Mat> &images) const;

 private: // 存储多个相机对象。cameras_ 是一个 std::vector，保存所有添加的 MEICamera 实例，用于处理多相机输入。
  std::vector<MEICamera> cameras_;

  int ipm_img_h_ = 1000; // 鸟瞰视图图像的高度
  int ipm_img_w_ = 1000; // 鸟瞰视图图像的宽度
  const double pixel_scale_ = 0.02; // 定义鸟瞰图像的像素比例。pixel_scale_ 表示每个像素对应的物理距离（单位：米/像素，0.02 米 = 2 厘米）。用于将地面坐标映射到图像像素。
  const double ipm_img_height_ = 0.0; // 定义鸟瞰图像的参考平面高度。ipm_img_height_ = 0.0 表示鸟瞰视图假设地面平面位于（世界坐标系的高度）。这是 IPM 的核心假设，确保映射到地面平面
};