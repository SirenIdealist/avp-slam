//
// Created by ubuntu on 2024-12-31.
// Tong Qin: qintong@sjtu.edu.cn
//

// 引入ROS、消息类型、Eigen库、可视化、TF变换和随机数生成等相关头文件
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <eigen3/Eigen/Geometry>
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <random>

// 定义ROS话题发布器和路径消息变量
ros::Publisher pub_estimation_path, pub_gt_path, pub_measurement_path, pub_odometry;
nav_msgs::Path path_estimation, path_gt, path_measurment;
ros::Publisher meshPub;

// 定义运动和观测噪声的标准差parameters for noise
double v_std = 0.5;
double yaw_rate_std = 0.1;
double x_std = 0.5;
double y_std = 0.5;
double yaw_std = 5.0 / 180. * M_PI;

// 定义高斯分布，用于生成噪声
std::default_random_engine generator;
std::normal_distribution<double> dist_x(0, x_std);
std::normal_distribution<double> dist_y(0, y_std);
std::normal_distribution<double> dist_yaw(0, yaw_std);
std::normal_distribution<double> dist_v(0, v_std);
std::normal_distribution<double> dist_yaw_rate(0, yaw_rate_std);

// 定义运动噪声和观测噪声的协方差矩阵
Eigen::Matrix2d Qn;
Eigen::Matrix3d Rn;


// 定义状态结构体，包含车辆状态，包括时间、位置、航向角和协方差
struct State {
    double time;
    double x;
    double y;
    double yaw;
    Eigen::Matrix3d P;
};

// 发布路径函数
void PublishPath(const double &time, const double &x, const double &y,
                 const double &yaw,
                 nav_msgs::Path &path,
                 ros::Publisher &path_publisher) {
    // 将二维位置和航向角转换为三维位姿，并发布到ROS路径话题
    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
    Eigen::Matrix3d R;
    R << cos(yaw), - sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    Eigen::Quaterniond q(R);

    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(time);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();

    // pub path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(time);
    pose_stamped.pose = odometry.pose.pose;
    path.poses.push_back(pose_stamped);
    if (path.poses.size() > 10000) {
        path.poses.erase(path.poses.begin());
    }
    path_publisher.publish(path);
}

// 发布路径和TF变换及Mesh模型
void PublishPathAndTF(const double &time, const double &x, const double &y,
             const double &yaw,
             nav_msgs::Path &path,
             ros::Publisher &path_publisher) {
    PublishPath(time, x, y, yaw, path, path_publisher);
    // convert 2D x, y, yaw to 3D x, y, z and rotation matrix
    Eigen::Vector3d position = Eigen::Vector3d(x, y, 0);
    Eigen::Matrix3d R;
    R << cos(yaw), - sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
    Eigen::Quaterniond q(R);

    // 发布路径，并广播TF变换，同时发布车辆Mesh模型用于可视化
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion tf_q;
    transform.setOrigin(tf::Vector3(position(0),
                                    position(1),
                                    position(2)));
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    transform.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(time),
                                          "world", "vehicle"));

    // pub Mesh model
    visualization_msgs::Marker meshROS;
    meshROS.header.frame_id = std::string("world");
    meshROS.header.stamp = ros::Time(time);
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    Eigen::Matrix3d rot_mesh;
    rot_mesh << -1, 0, 0, 0, 0, 1, 0, 1, 0;
    Eigen::Quaterniond q_mesh;
    q_mesh = q * rot_mesh;
    Eigen::Vector3d t_mesh = R * Eigen:: Vector3d(1.5, 0, 0) + position;
    meshROS.pose.orientation.w = q_mesh.w();
    meshROS.pose.orientation.x = q_mesh.x();
    meshROS.pose.orientation.y = q_mesh.y();
    meshROS.pose.orientation.z = q_mesh.z();
    meshROS.pose.position.x = t_mesh(0);
    meshROS.pose.position.y = t_mesh(1);
    meshROS.pose.position.z = t_mesh(2);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 1.0;
    meshROS.color.r = 1.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    std::string mesh_resource = "package://vehicle_state_estimation/meshes/car.dae"; // change this to your own mesh path
    meshROS.lifetime = ros::Duration(1.0);
    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);
}

// 生成轨迹点,根据时间生成车辆的理想轨迹点和航向角
void GeneratePose(const double &time, double &x, double &y, double &yaw) {
    x = 10 * sin(time / 10 * M_PI);
    y = 10 * cos(time / 10 * M_PI) - 10;
    double vx = (1.0 / 10 * M_PI) * 10 * cos(time / 10 * M_PI);
    double vy = (1.0 / 10 * M_PI) * 10 * (-1) * sin(time / 10 * M_PI);
    yaw = atan2(vy , vx);
}

// 扩展卡尔曼滤波的预测步骤
void EkfPredict(State& state, const double &time, const double &velocity, const double &yaw_rate) {
    // 步长与主循环一致
    double dt = 0.01;

    // 当前状态
    double x = state.x;
    double y = state.y;
    double yaw = state.yaw;

    // 1. 状态预测（运动模型）
    double x_pred = x + velocity * cos(yaw) * dt;
    double y_pred = y + velocity * sin(yaw) * dt;
    double yaw_pred = yaw + yaw_rate * dt;

    // 保证yaw在[-pi, pi]
    if (yaw_pred > M_PI) yaw_pred -= 2 * M_PI;
    if (yaw_pred < -M_PI) yaw_pred += 2 * M_PI;

    // 2. 计算雅可比矩阵F（对状态求偏导）
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -velocity * sin(yaw) * dt;
    F(1,2) =  velocity * cos(yaw) * dt;

    // 3. 计算控制噪声输入矩阵L
    Eigen::Matrix<double, 3, 2> L = Eigen::Matrix<double, 3, 2>::Zero();
    L(0,0) = cos(yaw) * dt;
    L(1,0) = sin(yaw) * dt;
    L(2,1) = dt;

    // 4. 协方差预测
    state.P = F * state.P * F.transpose() + L * Qn * L.transpose();

    // 5. 更新状态
    state.x = x_pred;
    state.y = y_pred;
    state.yaw = yaw_pred;
    state.time = time;
}

// EKF更新函数
void EkfUpdate(State& state,  const double &m_x, const double &m_y, const double &m_yaw) {
    // 观测模型 H
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    // 观测向量
    Eigen::Vector3d z;
    z << m_x, m_y, m_yaw;

    // 预测观测
    Eigen::Vector3d z_pred;
    z_pred << state.x, state.y, state.yaw;

    // 观测残差
    Eigen::Vector3d y = z - z_pred;
    // 保证yaw误差在[-pi, pi]
    if (y(2) > M_PI) y(2) -= 2 * M_PI;
    if (y(2) < -M_PI) y(2) += 2 * M_PI;

    // 卡尔曼增益
    Eigen::Matrix3d S = H * state.P * H.transpose() + Rn;
    Eigen::Matrix3d K = state.P * H.transpose() * S.inverse();

    // 状态更新
    Eigen::Vector3d dx = K * y;
    state.x += dx(0);
    state.y += dx(1);
    state.yaw += dx(2);
    // 保证yaw在[-pi, pi]
    if (state.yaw > M_PI) state.yaw -= 2 * M_PI;
    if (state.yaw < -M_PI) state.yaw += 2 * M_PI;

    // 协方差更新
    state.P = (Eigen::Matrix3d::Identity() - K * H) * state.P;
}


// 获取运动信号（带噪声）,通过差分计算速度和航向角速度，并加入高斯噪声。
void GetMotionSignal(const double &time, double &velocity, double &yaw_rate) {
    double x0, y0, yaw0;
    double t1 = time + 1e-6;
    double x1, y1, yaw1;
    GeneratePose(time, x0, y0, yaw0);
    GeneratePose(t1, x1, y1, yaw1);
    double vx = (x1 - x0) / 1e-6;
    double vy = (y1 - y0) / 1e-6;
    double dyaw = yaw1 - yaw0;
    if (dyaw > M_PI)
        dyaw -= 2 * M_PI;
    if (dyaw < -M_PI)
        dyaw += 2 * M_PI;
    yaw_rate = dyaw / 1e-6;
    velocity = sqrt(vx * vx + vy * vy);

    // add Gaussian noise
    velocity += dist_v(generator);
    yaw_rate += dist_yaw_rate(generator);
}


// 获取观测值（带噪声）, 生成带噪声的观测值
void GetPoseMeasurement(const double &time, double &mx, double &my, double &myaw) {
    GeneratePose(time, mx, my, myaw);

    // add Gaussian noise
    mx += dist_x(generator);
    my += dist_y(generator);
    myaw += dist_yaw(generator);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_state_estimation");
    ros::NodeHandle n("~");

    pub_estimation_path = n.advertise<nav_msgs::Path>("path_estimation", 1000);
    pub_gt_path = n.advertise<nav_msgs::Path>("path_gt", 1000);
    pub_measurement_path = n.advertise<nav_msgs::Path>("path_measurement", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    meshPub   = n.advertise<visualization_msgs::Marker>("vehicle", 100, true);
    path_estimation.header.frame_id = "world";
    path_gt.header.frame_id = "world";
    path_measurment.header.frame_id = "world";

    // variance of motion noise [v, yaw_rate]
    Qn << v_std * v_std, 0,
          0, yaw_rate_std * yaw_rate_std;

    // variance of measurement noise [x, y, yaw]
    Rn << x_std * x_std, 0, 0,
            0, y_std * y_std, 0,
            0, 0, yaw_std * yaw_std;

            
    // 初始化发布器
    // 初始化路径消息
    // 初始化噪声协方差矩阵

    // 初始化状态
    State state;
    state.x = 0;
    state.y = 0;
    state.yaw = 0;
    state.P = 10 * 10 * Eigen::Matrix3d::Identity();
    state.time = 0;

    ros::Rate loop_rate(100);
    double time = 0;
    int cnt = 0;

    while (ros::ok())
    {
        // predict at 100hz
        double velocity, yaw_rate = 0;
        GetMotionSignal(time, velocity, yaw_rate);
        // Ekf predict
        EkfPredict(state, time, velocity, yaw_rate);

        // update at 10hz
        if (cnt % 10 == 0) {
            double mx, my, myaw = 0;
            GetPoseMeasurement(time, mx, my, myaw);
            // publish raw measurement
            PublishPath(time, mx, my, myaw, path_measurment, pub_measurement_path);
            // Ekf update
            EkfUpdate(state, mx, my, myaw);
        }

        // publish estimation result
        PublishPathAndTF(time, state.x, state.y, state.yaw, path_estimation, pub_estimation_path);

        // generate and publish ground truth
        double x_gt = 0, y_gt = 0, yaw_gt = 0;
        GeneratePose(time, x_gt, y_gt, yaw_gt);
        PublishPath(time, x_gt, y_gt, yaw_gt, path_gt, pub_gt_path);

        ros::spinOnce();
        loop_rate.sleep();

        time += 0.01;
        cnt++;
    }
    ros::spin();
}
