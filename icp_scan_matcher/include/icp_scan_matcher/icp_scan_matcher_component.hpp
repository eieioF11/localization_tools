#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <execution>
#include <mutex>
#include <optional>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "common_lib/ros2_utility/extension_node.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
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
// #define POINT_CLOUD_CHECH
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class ICPScanMatcher : public ExtensionNode
{
public:
	ICPScanMatcher(const rclcpp::NodeOptions &options) : ICPScanMatcher("", options) {}
	ICPScanMatcher(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("icp_scan_matcher_node", name_space, options), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start icp_scan_matcher_node");
		// get param
		std::string CLOUD_TOPIC = param<std::string>("cloud", "/camera/depth_registered/points");
		std::string ODOM_TOPIC = param<std::string>("odom", "/odom");
		// frame
		// FIELD_FRAME = param<std::string>("field_frame", "field");
		FIELD_FRAME = param<std::string>("field_frame", "base_link");
		ODOM_FRAME = param<std::string>("odom_frame", "odom");
		ROBOT_FRAME = param<std::string>("robot_frame", "base_link");
		// setup
		BROADCAST_PERIOD = param<double>("broadcast_period", 0.001);
		ODOM_TF = param<bool>("odom_tf_broadcast", true);
		// 点群パラメータ
		MIN_CLOUD_SIZE = param<int>("min_point_cloud_size", 100);
		VOXELGRID_SIZE = param<double>("voxelgrid_size", 0.06);
		// scan matchingパラメータ
		TARGET_VOXELGRID_SIZE = param<double>("target_voxelgrid_size", 0.1);
		TARGET_UPDATE_MIN_SCORE = param<double>("target_update_min_score", 0.0005);
		MIN_SCORE_LIMIT = param<double>("min_score_limit", 0.01);
		// kalman filter
		double POS_Q = param<double>("pos_Q", 0.2);
		double RPY_Q = param<double>("rpy_Q", 0.2);
		double POS_R = param<double>("pos_R", 0.7);
		double RPY_R = param<double>("rpy_R", 0.7);
		// grid point observe
		grid_point_observe_parameter_t gpo_param;
		gpo_param.grid_width = param<double>("grid_width", 0.01);
		gpo_param.min_gain_position = param<double>("min_gain_position", 0.1);
		gpo_param.min_gain_orientation = param<double>("min_gain_orientation", 0.03);
		gpo_param.max_gain_position = param<double>("max_gain_position", 0.4);
		gpo_param.max_gain_orientation = param<double>("max_gain_orientation", 0.1);
		// robotパラメータ
		MIN_VEL = param<double>("min_velocity", 0.01);
		MIN_ANGULAR = param<double>("min_angular", 0.01);
		MAX_VEL = param<double>("max_velocity", 5.0);
		gpo_param.max_angular = param<double>("max_angular", 5.0);
		// 外れ値
		OUTLIER_DIST = param<double>("outlier_distance", 5.0);
		// init
		gpo_param.max_velocity = MAX_VEL;
		gpo_.set_params(gpo_param);
		odom_pose_.position = laser_pose_.position = estimate_pose_.position = {0.0, 0.0, 0.0};
		odom_pose_.orientation = laser_pose_.orientation = estimate_pose_.orientation = {0.0, 0.0, 0.0, 1.0};
		laser_pose_msg_.pose.orientation.w = 1.0;
		initialization_ = true;
		update_cloud_ = false;
		pos_klf_.F = Eigen::Vector3d::Ones().asDiagonal();
		Eigen::Vector3d g_vec, q_vec, r_vec;
		g_vec << BROADCAST_PERIOD, BROADCAST_PERIOD, BROADCAST_PERIOD;
		pos_klf_.G = g_vec.asDiagonal();
		pos_klf_.H = Eigen::Vector3d::Ones().asDiagonal();
		q_vec << POS_Q, POS_Q, POS_Q;
		pos_klf_.Q = q_vec.asDiagonal();
		r_vec << POS_R, POS_R, POS_R;
		pos_klf_.R = r_vec.asDiagonal();

		rpy_klf_.F = Eigen::Vector3d::Ones().asDiagonal();
		g_vec << BROADCAST_PERIOD, BROADCAST_PERIOD, BROADCAST_PERIOD;
		rpy_klf_.G = g_vec.asDiagonal();
		rpy_klf_.H = Eigen::Vector3d::Ones().asDiagonal();
		q_vec << RPY_Q, RPY_Q, RPY_Q;
		rpy_klf_.Q = q_vec.asDiagonal();
		r_vec << RPY_R, RPY_R, RPY_R;
		rpy_klf_.R = r_vec.asDiagonal();
		// publisher
		laser_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("icp_scan_matcher/laser_pose", rclcpp::QoS(10).best_effort());
		debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/debug_points", rclcpp::QoS(10));
		now_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/now_points", rclcpp::QoS(10));
		old_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/old_points", rclcpp::QoS(10));
		icp_final_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("icp_scan_matcher/icp_final_points", rclcpp::QoS(10));
		// subscriber
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, rclcpp::QoS(10), std::bind(&ICPScanMatcher::odometry_callback, this, std::placeholders::_1));
		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(CLOUD_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&ICPScanMatcher::pointcloud_callback, this, std::placeholders::_1));
		// timer
		main_timer_ = this->create_wall_timer(1ms, [&]() {});
	}

	// callback
	void odometry_callback(const nav_msgs::msg::Odometry::ConstPtr msg)
	{
	}

	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg)
	{
		static uint64_t num = 1;
		static long double sum = 0.;
		static long double ssum = 0.;
#if defined(DEBUG_OUTPUT)
		std::cout << "----------------------------------------------------------------------------------" << std::endl;
		std::cout << "get cloud" << std::endl;
#endif
		sensor_msgs::msg::PointCloud2 get_cloud;
		get_cloud = *msg;
		// std::string CAMERA_FRAME = "mercury/camera_link";
		// get_cloud.header.frame_id = CAMERA_FRAME;
		debug_cloud_pub_->publish(get_cloud);
		std::optional<sensor_msgs::msg::PointCloud2> trans_cloud = transform_pointcloud2(tf_buffer_, FIELD_FRAME, get_cloud);
		if (trans_cloud)
		{
			// msg convert
			pcl::PointCloud<pcl::PointXYZ> now_cloud;
			pcl::fromROSMsg(trans_cloud.value(), now_cloud);
			if (!now_cloud.empty())
			{
				//  Down sampling
				now_cloud = voxelgrid_filter(now_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
				now_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), now_cloud));
				if (initialization_)
				{
#if defined(DEBUG_OUTPUT)
					std::cout << "Initialization" << std::endl;
					std::cout << "now_cloud size:" << now_cloud.points.size() << "|old_cloud size:" << old_cloud_.points.size() << std::endl;
#endif
					// nan値除去
					std::vector<int> mapping;
					pcl::removeNaNFromPointCloud(now_cloud, now_cloud, mapping);
					old_cloud_ = now_cloud;

					sum = 0.;
					ssum = 0.;
					num = 1;
					initialization_ = false;
				}
				else if (!old_cloud_.empty())
				{
#if defined(DEBUG_OUTPUT)
					std::cout << "now_cloud size:" << now_cloud.points.size() << "|old_cloud size:" << old_cloud_.points.size() << std::endl;
#endif
					if (now_cloud.points.size() > MIN_CLOUD_SIZE)
					{
						// nan値除去
						std::vector<int> mapping1, mapping2;
						pcl::removeNaNFromPointCloud(now_cloud, now_cloud, mapping1);
						pcl::removeNaNFromPointCloud(old_cloud_, old_cloud_, mapping2);
						old_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), old_cloud_));

#if defined(POINT_CLOUD_CHECH)
						// nan値がないことのチェック
						pcl::DefaultPointRepresentation<pcl::PointXYZ> representation;
						std::cout << "old cloud" << std::endl;
						for (int i = 0; i < old_cloud_.points.size(); ++i)
						{
							if (!representation.isValid(old_cloud_.points[i]))
								std::cout << "invalid1 " << i << " " << old_cloud_.points[i] << std::endl;
						}
						std::cout << "now cloud" << std::endl;
						for (int i = 0; i < now_cloud.points.size(); ++i)
						{
							if (!representation.isValid(now_cloud.points[i]))
								std::cout << "invalid2 " << i << " " << now_cloud.points[i] << std::endl;
						}
#endif

						// ICP
						const auto &result = iterative_closest_point(now_cloud, old_cloud_);
						if (result)
						{
							auto &[score, tmat, final_cloud] = result.value();
							icp_final_cloud_pub_->publish(make_ros_pointcloud2(make_header(FIELD_FRAME, get_cloud.header.stamp), final_cloud));
#if defined(DEBUG_OUTPUT)
							std::cout << "ICP has converged, score is " << score << std::endl;
#endif
							if (score < MIN_SCORE_LIMIT)
							{
#if defined(ICP_RESULT)
								std::cout << "rotation matrix" << std::endl;
								for (int i = 0; i < 3; ++i)
								{
									std::cout << tmat(i, 0) << ", " << tmat(i, 1) << ", " << tmat(i, 2) << std::endl;
								}
								std::cout << std::endl;
								std::cout << "translation vector" << std::endl;
								std::cout << tmat(0, 3) << ", " << tmat(1, 3) << ", " << tmat(2, 3) << std::endl;
#endif
								// calc laser position
								laser_pose_ = estimate_pose_;
								Vector3d rpy = laser_pose_.orientation.get_rpy();
								Vector3d diff_rpy = {std::atan2(tmat(2, 1), tmat(2, 2)), -std::asin(tmat(2, 0)), std::atan2(tmat(1, 0), tmat(0, 0))};
								Vector3d diff_pos = {tmat(0, 3), tmat(1, 3), tmat(2, 3)};

								// 一つ前の推定値との距離計算
								double delta_dist = Vector3d::distance(diff_pos, old_diff_pos_);
								old_diff_pos_ = diff_pos;
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
								if (mahalanobis_dist < OUTLIER_DIST)
								{
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
									double dt = (rclcpp::Clock().now() - pre_time).seconds();
									pre_time = rclcpp::Clock().now();
									if (approx_zero(dt))
									{
										Eigen::Vector3d g_vec;
										g_vec << dt, dt, dt;
										pos_klf_.G = g_vec.asDiagonal();
										rpy_klf_.G = g_vec.asDiagonal();
									}
									Eigen::Vector3d u, z;
									u << odom_linear_.x, odom_linear_.y, odom_linear_.z;
									z << laser_pose_.position.x, laser_pose_.position.y, laser_pose_.position.z;
									auto x = pos_klf_.filtering(u, z);
									laser_pose_.position = {x(0), x(1), x(2)};

									u << odom_angular_.x, odom_angular_.y, odom_angular_.z;
									z << rpy.x, rpy.y, rpy.z;
									x = rpy_klf_.filtering(u, z);
									laser_pose_.orientation.set_rpy(x(0), x(1), x(2));
									// grid point observe
									gpo_.set_starreckoning(laser_pose_.position, {x(0), x(1), x(2)});
									// update point cloud
									double velocity = odom_linear_.norm(); // 並進速度の大きさ//std::hypot(odom_linear_.x, odom_linear_.y, odom_linear_.z);
									double angular = odom_angular_.norm(); // 回転速度の大きさ//std::hypot(odom_angular_.x, odom_angular_.y, odom_angular_.z);
									if (angular < MIN_ANGULAR)			   // 並進移動時のみ更新する
									{
										if (velocity > MIN_VEL)
											update_cloud_ = false;
										if (velocity < MAX_VEL)
											update_target_cloud(score, final_cloud);
										update_cloud_ = false; //
									}
									else
										update_cloud_ = false;
#if defined(DEBUG_OUTPUT)
									std::cout << "velocity " << velocity << "|angular " << angular << std::endl;
									std::cout << "laser pose" << std::endl;
									std::cout << laser_pose_ << std::endl;
#endif
								}
							}
						}
						else
							RCLCPP_ERROR(this->get_logger(), "ICP has not converged.");
					}
				}
				else
					old_cloud_ = now_cloud;
			}
			else
				RCLCPP_ERROR(this->get_logger(), "now_cloud empty");
		}
		else
			RCLCPP_ERROR(this->get_logger(), "transform error");
		// debug output
		// laser_pose_msg_.header = make_header(ODOM_FRAME, rclcpp::Clock().now());
		laser_pose_msg_.header = make_header(FIELD_FRAME, rclcpp::Clock().now());
		laser_pose_msg_.pose = make_geometry_pose(laser_pose_);
		laser_pose_pub_->publish(laser_pose_msg_);
	}

private:
	bool initialization_;
	bool update_cloud_;
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
	bool ODOM_TF;
	// マルチスレッド関連
	inline static std::mutex mtx;
	// tf
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listener_;
	// timer
	rclcpp::TimerBase::SharedPtr main_timer_;
	// subscriber
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
	Pose3d init_odom_pose_;
	Pose3d estimate_pose_;
	geometry_msgs::msg::PoseStamped laser_pose_msg_;

	Vector3d old_diff_pos_;
	// Vector3d old_diff_rpy_;
	// vel
	Vector3d odom_linear_;
	Vector3d odom_angular_;

	// function

	void update_target_cloud(double score, const pcl::PointCloud<pcl::PointXYZ> &input_cloud)
	{
		if (!update_cloud_)
		{
			if (score < TARGET_UPDATE_MIN_SCORE)
			{
				old_cloud_ += input_cloud;
				//  Down sampling
				old_cloud_ = voxelgrid_filter(old_cloud_, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE, TARGET_VOXELGRID_SIZE);
				update_cloud_ = true;
			}
		}
	}
};