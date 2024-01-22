#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
// #include "common_lib/ros2_utility/marker_util.hpp"
// PCL
#include "common_lib/pcl_utility/pcl_util.hpp"
// other
#include "common_lib/common_lib.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// for文の実行方法設定
//  #define LOOP_MODE std::execution::seq // 逐次実行
//  #define LOOP_MODE std::execution::par // 並列実行
#define LOOP_MODE std::execution::par_unseq // 並列化・ベクトル化
// デバック関連設定
#define DEBUG_OUTPUT
// #define ICP_RESULT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class ICPScanMatcher : public ExtensionNode {
public:
  ICPScanMatcher(const rclcpp::NodeOptions& options) : ICPScanMatcher("", options) {}
  ICPScanMatcher(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : ExtensionNode("icp_scan_matcher_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_) {
    RCLCPP_INFO(this->get_logger(), "start icp_scan_matcher_node");
    // get param
    std::string CLOUD_TOPIC = param<std::string>("icp_scan_matcher.topic_name.cloud", "/camera/depth_registered/points");
    std::string ODOM_TOPIC  = param<std::string>("icp_scan_matcher.topic_name.odom", "/odom");
    std::string IMU_TOPIC   = param<std::string>("icp_scan_matcher.topic_name.imu", "/wit_ros/imu_pose");
    // frame
    FIELD_FRAME = param<std::string>("icp_scan_matcher.tf_frame.map_frame", "map");
    ODOM_FRAME  = param<std::string>("icp_scan_matcher.tf_frame.odom_frame", "odom");
    ROBOT_FRAME = param<std::string>("icp_scan_matcher.tf_frame.robot_frame", "base_link");
    // setup
    BROADCAST_PERIOD = param<double>("icp_scan_matcher.broadcast_period", 0.001);
    ODOM_TF          = param<bool>("icp_scan_matcher.odom_tf_broadcast", true);
    // 点群パラメータ
    MIN_CLOUD_SIZE = param<int>("icp_scan_matcher.min_point_cloud_size", 100);
    VOXELGRID_SIZE = param<double>("icp_scan_matcher.filter.voxelgrid_size", 0.04);
    // radiusoutlierフィルタ
    RADIUS_SEARCH           = param<double>("icp_scan_matcher.filter.radius_search", 0.02);
    MIN_NEIGHBORS_IN_RADIUS = param<double>("icp_scan_matcher.filter.min_neighbors_in_radius", 5.0);
    // scan matchingパラメータ
    TARGET_VOXELGRID_SIZE   = param<double>("icp_scan_matcher.filter.target_voxelgrid_size", 0.04);
    TARGET_UPDATE_MIN_SCORE = param<double>("icp_scan_matcher.filter.target_update_min_score", 0.0005);
    MIN_SCORE_LIMIT         = param<double>("icp_scan_matcher.min_score_limit", 0.01);
    // kalman filter
    std::vector<double> K_Q = param<std::vector<double>>("icp_scan_matcher.kalman_filter.Q", {0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    std::vector<double> K_R = param<std::vector<double>>("icp_scan_matcher.kalman_filter.R", {0.7, 0.7, 0.7, 0.7, 0.7, 0.7});
    // grid point observe
    grid_point_observer_parameter_t gpo_param;
    gpo_param.grid_width           = param<double>("icp_scan_matcher.gpo.grid_width", 0.01);
    gpo_param.min_gain_position    = param<double>("icp_scan_matcher.gpo.min_gain_position", 0.1);
    gpo_param.min_gain_orientation = param<double>("icp_scan_matcher.gpo.min_gain_orientation", 0.03);
    gpo_param.max_gain_position    = param<double>("icp_scan_matcher.gpo.max_gain_position", 0.4);
    gpo_param.max_gain_orientation = param<double>("icp_scan_matcher.gpo.max_gain_orientation", 0.1);
    // robotパラメータ
    MIN_VEL               = param<double>("icp_scan_matcher.robot.min_velocity", 0.01);
    MIN_ANGULAR           = param<double>("icp_scan_matcher.robot.min_angular", 0.01);
    MAX_VEL               = param<double>("icp_scan_matcher.robot.max_velocity", 5.0);
    gpo_param.max_angular = param<double>("icp_scan_matcher.robot.max_angular", 5.0);
    // 外れ値
    OUTLIER_DIST = param<double>("icp_scan_matcher.outlier_distance", 5.0);
    // init
    get_imu_data_          = false;
    gpo_param.max_velocity = MAX_VEL;
    gpo_.set_params(gpo_param);
    odom_pose_.position = laser_pose_.position = estimate_pose_.position = {0.0, 0.0, 0.0};
    odom_pose_.orientation = laser_pose_.orientation = estimate_pose_.orientation = {0.0, 0.0, 0.0, 1.0};
    laser_pose_msg_.pose.orientation.w                                            = 1.0;
    initialization_                                                               = true;
    update_cloud_                                                                 = false;
    pos_klf_.F                                                                    = Eigen::Vector3d::Ones().asDiagonal();
    Eigen::Vector3d g_vec, q_vec, r_vec;
    g_vec << BROADCAST_PERIOD, BROADCAST_PERIOD, BROADCAST_PERIOD;
    pos_klf_.G = g_vec.asDiagonal();
    pos_klf_.H = Eigen::Vector3d::Ones().asDiagonal();
    q_vec << K_Q[0], K_Q[1], K_Q[2];
    pos_klf_.Q = q_vec.asDiagonal();
    r_vec << K_R[0], K_R[1], K_R[2];
    pos_klf_.R = r_vec.asDiagonal();

    rpy_klf_.F = Eigen::Vector3d::Ones().asDiagonal();
    g_vec << BROADCAST_PERIOD, BROADCAST_PERIOD, BROADCAST_PERIOD;
    rpy_klf_.G = g_vec.asDiagonal();
    rpy_klf_.H = Eigen::Vector3d::Ones().asDiagonal();
    q_vec << K_Q[3], K_Q[4], K_Q[5];
    rpy_klf_.Q = q_vec.asDiagonal();
    r_vec << K_R[3], K_R[4], K_R[5];
    rpy_klf_.R = r_vec.asDiagonal();
    // publisher
    laser_pose_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_scan_matcher/laser_pose", rclcpp::QoS(10).best_effort());
    debug_cloud_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/debug_points", rclcpp::QoS(10));
    now_cloud_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/now_points", rclcpp::QoS(10));
    old_cloud_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/old_points", rclcpp::QoS(10));
    icp_final_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/icp_final_points", rclcpp::QoS(10));
    // subscriber
    odom_sub_  = this->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, rclcpp::QoS(10),
                                                                   std::bind(&ICPScanMatcher::odometry_callback, this, std::placeholders::_1));
    imu_sub_   = this->create_subscription<geometry_msgs::msg::PoseStamped>(IMU_TOPIC, rclcpp::QoS(10).best_effort(),
                                                                          std::bind(&ICPScanMatcher::imu_callback, this, std::placeholders::_1));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        CLOUD_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&ICPScanMatcher::pointcloud_callback, this, std::placeholders::_1));
    // timer
    main_timer_ = this->create_wall_timer(1ms, [&]() {
      // grid point observe
      const auto& [estimate_position, estimate_orientation] = gpo_.update_estimate_pose();
      estimate_pose_.position                               = estimate_position;
      estimate_pose_.orientation.set_rpy(estimate_orientation.x, estimate_orientation.y, estimate_orientation.z);
      // map -> odom tf
      if (ODOM_TF) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header         = make_header(FIELD_FRAME, rclcpp::Clock().now());
        transform_stamped.child_frame_id = ODOM_FRAME;
        transform_stamped.transform      = make_geometry_transform(odom_pose_);
        broadcaster_.sendTransform(transform_stamped);
      }
      // map -> base_link tf
      if (estimate_pose_.position.has_nan() || estimate_pose_.orientation.has_nan()) {
        estimate_pose_.position    = {0.0, 0.0, 0.0};
        estimate_pose_.orientation = {0.0, 0.0, 0.0, 1.0};
      }

      estimate_pose_.orientation = imu_pose_.orientation;
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header         = make_header(FIELD_FRAME, rclcpp::Clock().now());
      transform_stamped.child_frame_id = ROBOT_FRAME;
      transform_stamped.transform      = make_geometry_transform(estimate_pose_);
      broadcaster_.sendTransform(transform_stamped);
    });
  }

  // callback
  void odometry_callback(const nav_msgs::msg::Odometry::ConstPtr msg) {}

  void imu_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg) {
    get_imu_data_ = true;
    imu_pose_     = make_pose(msg->pose);
    gpo_.set_deadreckoning({0.0, 0.0, 0.0}, imu_pose_.orientation.get_rpy(), {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg) {
    static uint64_t num     = 1;
    static long double sum  = 0.;
    static long double ssum = 0.;
#if defined(DEBUG_OUTPUT)
    std::cout << "----------------------------------------------------------------------------------" << std::endl;
    std::cout << "get cloud" << std::endl;
#endif
    sensor_msgs::msg::PointCloud2 get_cloud;
    get_cloud                                                = *msg;
    std::optional<sensor_msgs::msg::PointCloud2> trans_cloud = transform_pointcloud2(tf_buffer_, FIELD_FRAME, get_cloud);
    if (trans_cloud) {
      // msg convert
      pcl::PointCloud<pcl::PointXYZ> now_cloud;
      pcl::fromROSMsg(trans_cloud.value(), now_cloud);
      if (!now_cloud.empty()) {
        // nan値除去
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(now_cloud, now_cloud, mapping);
        //  Down sampling
        now_cloud = voxelgrid_filter(now_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
        now_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), now_cloud));
        // 外れ値除去
        // now_cloud = radiusoutlier_filter(now_cloud, RADIUS_SEARCH, MIN_NEIGHBORS_IN_RADIUS);
        debug_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), now_cloud));
#if defined(DEBUG_OUTPUT)
        std::cout << "now_cloud size:" << now_cloud.points.size() << "|old_cloud size:" << old_cloud_.points.size() << std::endl;
#endif
        if (now_cloud.points.size() > MIN_CLOUD_SIZE) {
          if (!old_cloud_.empty()) {
            old_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), old_cloud_));
            // ICP
            const auto& result = iterative_closest_point(now_cloud, old_cloud_);
            if (result) {
              auto& [score, tmat, final_cloud] = result.value();
              icp_final_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), final_cloud));
#if defined(DEBUG_OUTPUT)
              std::cout << "ICP has converged, score is " << score << std::endl;
#endif
              if (score < MIN_SCORE_LIMIT) {
#if defined(ICP_RESULT)
                std::cout << "rotation matrix" << std::endl;
                for (int i = 0; i < 3; ++i) {
                  std::cout << tmat(i, 0) << ", " << tmat(i, 1) << ", " << tmat(i, 2) << std::endl;
                }
                std::cout << std::endl;
                std::cout << "translation vector" << std::endl;
                std::cout << tmat(0, 3) << ", " << tmat(1, 3) << ", " << tmat(2, 3) << std::endl;
#endif
                // calc laser position
                laser_pose_       = estimate_pose_;
                Vector3d rpy      = laser_pose_.orientation.get_rpy();
                Vector3d diff_rpy = {std::atan2(tmat(2, 1), tmat(2, 2)), -std::asin(tmat(2, 0)), std::atan2(tmat(1, 0), tmat(0, 0))};
                Vector3d diff_pos = {tmat(0, 3), tmat(1, 3), tmat(2, 3)};

                // 一つ前の推定値との距離計算
                double delta_dist = Vector3d::distance(diff_pos, old_diff_pos_);
                old_diff_pos_     = diff_pos;
                // 外れ値判定
                // 平均値計算
                double ave_delta_dist = sum / (double)num;
                // 標準偏差計算
                double std_delta_dist = std::sqrt(ssum / (double)num - ave_delta_dist * ave_delta_dist);
                if (approx_zero(std_delta_dist)) // 0除算回避
                  std_delta_dist = 1.0;
                // マハラノビス距離
                double mahalanobis_dist = std::abs(delta_dist - ave_delta_dist) / std_delta_dist;
#if defined(DEBUG_OUTPUT)
                std::cout << "delta_dist:" << delta_dist << std::endl;
                std::cout << "ave_delta_dist:" << ave_delta_dist << std::endl;
                std::cout << "std_delta_dist:" << std_delta_dist << std::endl;
                std::cout << "mahalanobis_dist:" << mahalanobis_dist << std::endl;
#endif
                if (mahalanobis_dist < OUTLIER_DIST) {
                  sum += delta_dist;
                  ssum += delta_dist * delta_dist;
                  num++;

                  // double diff_t = (double)(rclcpp::Clock().now() - get_cloud.header.stamp).seconds();
                  // Vector3d corr_pos = {odom_linear_.x * diff_t, odom_linear_.y * diff_t, odom_linear_.z * diff_t};
                  // Vector3d corr_rpy = {odom_angular_.x * diff_t, odom_angular_.y * diff_t, odom_angular_.z * diff_t};
                  // diff_pos += corr_pos;
                  // diff_rpy += corr_rpy;

                  rpy += diff_rpy;
                  laser_pose_.position += diff_pos;
#if defined(DEBUG_OUTPUT)
                  std::cout << "diff_pos:" << diff_pos << "|diff_rpy:" << diff_rpy << std::endl;
                  std::cout << "pos:" << laser_pose_.position << "|rpy" << rpy << std::endl;
                  std::cout << "norm pos:" << diff_pos.norm() << "|rpy:" << diff_rpy.norm() << std::endl;
#endif
                  // filter
                  static rclcpp::Time pre_time = rclcpp::Clock().now();
                  double dt                    = (rclcpp::Clock().now() - pre_time).seconds();
                  pre_time                     = rclcpp::Clock().now();
                  if (approx_zero(dt)) {
                    Eigen::Vector3d g_vec;
                    g_vec << dt, dt, dt;
                    pos_klf_.G = g_vec.asDiagonal();
                    rpy_klf_.G = g_vec.asDiagonal();
                  }
                  Eigen::Vector3d u, z;
                  u << odom_linear_.x, odom_linear_.y, odom_linear_.z;
                  z << laser_pose_.position.x, laser_pose_.position.y, laser_pose_.position.z;
                  auto x               = pos_klf_.filtering(u, z);
                  laser_pose_.position = {x(0), x(1), x(2)};

                  u << odom_angular_.x, odom_angular_.y, odom_angular_.z;
                  z << rpy.x, rpy.y, rpy.z;
                  x = rpy_klf_.filtering(u, z);
                  laser_pose_.orientation.set_rpy(x(0), x(1), x(2));
                  // grid point observe
#if defined(DEBUG_OUTPUT)
                  std::cout << "imu:" << imu_pose_.orientation.get_rpy() << std::endl;
#endif
                  gpo_.set_starreckoning(laser_pose_.position, {x(0), x(1), x(2)});
                  // update point cloud
                  double velocity = odom_linear_.norm();  // 並進速度の大きさ//std::hypot(odom_linear_.x, odom_linear_.y, odom_linear_.z);
                  double angular  = odom_angular_.norm(); // 回転速度の大きさ//std::hypot(odom_angular_.x, odom_angular_.y, odom_angular_.z);
                  if (angular < MIN_ANGULAR)              // 並進移動時のみ更新する
                  {
                    update_cloud_ = false; //
                    if (velocity > MIN_VEL) update_cloud_ = false;
                    if (velocity < MAX_VEL) update_target_cloud(score, final_cloud);
                  } else
                    update_cloud_ = false;
#if defined(DEBUG_OUTPUT)
                  std::cout << "velocity " << velocity << "|angular " << angular << std::endl;
                  std::cout << "laser pose" << std::endl;
                  std::cout << laser_pose_ << std::endl;
#endif
                }
              }
            } else
              RCLCPP_ERROR(this->get_logger(), "ICP has not converged.");
          } else
            old_cloud_ = now_cloud;
        }
      } else
        RCLCPP_ERROR(this->get_logger(), "now_cloud empty");
    } else
      RCLCPP_ERROR(this->get_logger(), "transform error");
    // debug output
    laser_pose_msg_.header = make_header(FIELD_FRAME, rclcpp::Clock().now());
    laser_pose_msg_.pose   = make_geometry_pose(laser_pose_);
    laser_pose_pub_->publish(laser_pose_msg_);
  }

private:
  bool initialization_;
  bool update_cloud_;
  bool get_imu_data_;
  // param
  std::string FIELD_FRAME;
  std::string ROBOT_FRAME;
  std::string ODOM_FRAME;
  double BROADCAST_PERIOD;
  int MIN_CLOUD_SIZE;
  double VOXELGRID_SIZE;
  double TARGET_VOXELGRID_SIZE;
  double MIN_SCORE_LIMIT;
  double TARGET_UPDATE_MIN_SCORE;
  double MIN_VEL;
  double MIN_ANGULAR;
  double MAX_VEL;
  double OUTLIER_DIST;
  double RADIUS_SEARCH;
  double MIN_NEIGHBORS_IN_RADIUS;
  bool ODOM_TF;
  // マルチスレッド関連
  inline static std::mutex mtx;
  // tf
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr main_timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr laser_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr now_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr old_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_final_cloud_pub_;
  // filter
  KalmanFilter<double, 3, 3, 3> pos_klf_;
  KalmanFilter<double, 3, 3, 3> rpy_klf_;
  GridPointObserver gpo_;
  // cloud
  pcl::PointCloud<pcl::PointXYZ> old_cloud_;
  // pose
  Pose3d laser_pose_;
  Pose3d odom_pose_;
  Pose3d imu_pose_;
  Pose3d init_odom_pose_;
  Pose3d estimate_pose_;
  geometry_msgs::msg::PoseStamped laser_pose_msg_;

  Vector3d old_diff_pos_;
  // Vector3d old_diff_rpy_;
  // vel
  Vector3d odom_linear_;
  Vector3d odom_angular_;

  // function

  void update_target_cloud(double score, const pcl::PointCloud<pcl::PointXYZ>& input_cloud) {
    if (!update_cloud_) {
      if (score < TARGET_UPDATE_MIN_SCORE) {
        old_cloud_ += input_cloud;
        //  Down sampling
        old_cloud_    = voxelgrid_filter(old_cloud_, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE);
        update_cloud_ = true;
      }
    }
  }
};