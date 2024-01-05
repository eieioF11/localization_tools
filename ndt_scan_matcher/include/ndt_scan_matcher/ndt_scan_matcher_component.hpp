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

#include "extension_node/extension_node.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
// #include "common_lib/ros2_utility/marker_util.hpp"
// PCL
#define PCL_DEBUG_OUTPUT
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
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class NDTScanMatcher : public ExtensionNode
{
public:
	NDTScanMatcher(const rclcpp::NodeOptions &options) : NDTScanMatcher("", options) {}
	NDTScanMatcher(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("ndt_scan_matcher_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start ndt_scan_matcher_node");
		// get param
		std::string CLOUD_TOPIC = param<std::string>("ndt_scan_matcher.topic_name.cloud", "/camera/depth_registered/points");
		std::string ESTIMATE_POSE_TOPIC = param<std::string>("ndt_scan_matcher.topic_name.estimate_pose", "/estimate_pose");
		std::string ESTIMATE_TWIST_TOPIC = param<std::string>("ndt_scan_matcher.topic_name.estimate_twist", "/estimate_twist");
		// frame
		MAP_FRAME = param<std::string>("ndt_scan_matcher.tf_frame.map_frame", "map");
		ODOM_FRAME = param<std::string>("ndt_scan_matcher.tf_frame.odom_frame", "odom");
		ROBOT_FRAME = param<std::string>("ndt_scan_matcher.tf_frame.robot_frame", "base_link");
		// setup
		BROADCAST_PERIOD = param<double>("ndt_scan_matcher.broadcast_period", 0.001);
		// 点群パラメータ
		MIN_CLOUD_SIZE = param<int>("ndt_scan_matcher.min_point_cloud_size", 100);
		VOXELGRID_SIZE = param<double>("ndt_scan_matcher.filter.voxelgrid_size", 0.04);
		// scan matchingパラメータ
		ndt_param_.trans_epsilon = param<double>("ndt_scan_matcher.ndt.trans_epsilon", 0.00000001);
		ndt_param_.step_size = param<double>("ndt_scan_matcher.ndt.step_size", 0.1);
		ndt_param_.resolution = param<double>("ndt_scan_matcher.ndt.resolution", 0.1);
		ndt_param_.max_iterations = param<double>("ndt_scan_matcher.ndt.max_iterations", 100.0);
		MIN_SCORE_LIMIT = param<double>("ndt_scan_matcher.ndt.min_score_limit", 0.0001);
		// kalman filter
		std::vector<double> K_Q = param<std::vector<double>>("ndt_scan_matcher.kalman_filter.Q", {0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
		std::vector<double> K_R = param<std::vector<double>>("ndt_scan_matcher.kalman_filter.R", {0.7, 0.7, 0.7, 0.7, 0.7, 0.7});
		// init
		laser_pose_msg_.pose.orientation.w = 1.0;
		initialization_ = true;
		pos_klf_.F = Eigen::Vector3d::Ones().asDiagonal();
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
		laser_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_scan_matcher/laser_pose", rclcpp::QoS(10).best_effort());
		debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_scan_matcher/debug_points", rclcpp::QoS(10));
		now_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_scan_matcher/now_points", rclcpp::QoS(10));
		old_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_scan_matcher/old_points", rclcpp::QoS(10));
		final_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_scan_matcher/final_points", rclcpp::QoS(10));
		// subscriber
		est_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ESTIMATE_POSE_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&NDTScanMatcher::est_pose_callback, this, std::placeholders::_1));
		est_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ESTIMATE_TWIST_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&NDTScanMatcher::est_twist_callback, this, std::placeholders::_1));
		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(CLOUD_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&NDTScanMatcher::pointcloud_callback, this, std::placeholders::_1));
	}

	// callback

	void est_twist_callback(const geometry_msgs::msg::TwistStamped::ConstPtr msg)
	{
		estimate_twist_ = make_twist(msg->twist);
	}

	void est_pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg)
	{
		estimate_pose_ = make_pose(msg->pose);
	}

	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr msg)
	{
#if defined(DEBUG_OUTPUT)
		std::cout << "----------------------------------------------------------------------------------" << std::endl;
		std::cout << "get cloud" << std::endl;
#endif
		sensor_msgs::msg::PointCloud2 get_cloud;
		get_cloud = *msg;
		std::optional<sensor_msgs::msg::PointCloud2> trans_cloud = transform_pointcloud2(tf_buffer_, MAP_FRAME, get_cloud);
		if (!trans_cloud)
		{
			RCLCPP_ERROR(this->get_logger(), "transform error");
			return;
		}
		// msg convert
		pcl::PointCloud<pcl::PointXYZ> now_cloud;
		pcl::fromROSMsg(trans_cloud.value(), now_cloud);
		if (now_cloud.empty())
		{
			RCLCPP_WARN(this->get_logger(), "now_cloud empty");
			return;
		}
		// nan値除去
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(now_cloud, now_cloud, mapping);
		//  Down sampling
		now_cloud = voxelgrid_filter(now_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
		now_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), now_cloud));
		// debug_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), now_cloud));
#if defined(DEBUG_OUTPUT)
		std::cout << "now_cloud size:" << now_cloud.points.size() << "|old_cloud size:" << old_cloud_.points.size() << std::endl;
#endif
		if (now_cloud.points.size() < MIN_CLOUD_SIZE)
		{
			RCLCPP_WARN(this->get_logger(), "Small input point cloud size");
			return;
		}
		if (old_cloud_.empty())
		{
			old_cloud_ = now_cloud;
			RCLCPP_WARN(this->get_logger(), "old_cloud empty");
			return;
		}
		old_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), old_cloud_));
		// NDT
		Pose3d init_pose;
		init_pose.position = -estimate_pose_.position;
		const auto &result = normal_distributions_transform(now_cloud, old_cloud_, ndt_param_, init_pose);
		// const auto &result = normal_distributions_transform(now_cloud, old_cloud_, ndt_param_, estimate_pose_);
		if (!result)
		{
			RCLCPP_ERROR(this->get_logger(), "NDT has not converged.");
			return;
		}
		auto &[score, tmat, final_cloud] = result.value();
		final_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), final_cloud));
#if defined(DEBUG_OUTPUT)
		std::cout << "NDT has converged, score is " << score << std::endl;
#endif
		laser_pose_ = get_pose(tmat);
		Vector3d rpy = laser_pose_.orientation.get_rpy();
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
		u << estimate_twist_.linear.x, estimate_twist_.linear.y, estimate_twist_.linear.z;
		z << laser_pose_.position.x, laser_pose_.position.y, laser_pose_.position.z;
		auto x = pos_klf_.filtering(u, z);
		laser_pose_.position = {x(0), x(1), x(2)};

		u << estimate_twist_.angular.x, estimate_twist_.angular.y, estimate_twist_.angular.z;
		z << rpy.x, rpy.y, rpy.z;
		x = rpy_klf_.filtering(u, z);
		laser_pose_.orientation.set_rpy(x(0), x(1), x(2));
		// publish
		laser_pose_msg_.header = make_header(MAP_FRAME, rclcpp::Clock().now());
		laser_pose_msg_.pose = make_geometry_pose(laser_pose_);
		laser_pose_pub_->publish(laser_pose_msg_);
		// update point cloud
#if defined(DEBUG_OUTPUT)
		std::cout << "laser pose" << std::endl;
		std::cout << "pos:" << laser_pose_.position << "|rpy" << rpy << std::endl;
		std::cout << "quaternion:" << laser_pose_.orientation << std::endl;
#endif
		if (score < MIN_SCORE_LIMIT)
		{
			old_cloud_ += final_cloud;
			old_cloud_ = voxelgrid_filter(old_cloud_, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
		}
		else
			RCLCPP_WARN(this->get_logger(), "Exceeding score limits");
	}

private:
	bool initialization_;
	// param
	std::string MAP_FRAME;
	std::string ROBOT_FRAME;
	std::string ODOM_FRAME;
	double BROADCAST_PERIOD;
	int MIN_CLOUD_SIZE;
	double MIN_SCORE_LIMIT;
	double VOXELGRID_SIZE;
	ndt_parameter_t ndt_param_;
	// マルチスレッド関連
	inline static std::mutex mtx;
	// tf
	tf2_ros::TransformBroadcaster broadcaster_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener listener_;
	// timer
	rclcpp::TimerBase::SharedPtr main_timer_;
	// subscriber
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr est_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr est_twist_sub_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
	// publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr laser_pose_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr now_cloud_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr old_cloud_pub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr final_cloud_pub_;
	// filter
	KalmanFilter<double, 3, 3, 3> pos_klf_;
	KalmanFilter<double, 3, 3, 3> rpy_klf_;
	// cloud
	pcl::PointCloud<pcl::PointXYZ> old_cloud_;
	// pose
	Pose3d laser_pose_;
	Pose3d estimate_pose_;
	geometry_msgs::msg::PoseStamped laser_pose_msg_;
	// vel
	Twistd estimate_twist_;
};