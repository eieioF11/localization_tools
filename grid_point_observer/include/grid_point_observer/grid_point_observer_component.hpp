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

#include "extension_node/extension_node.hpp"
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
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class GridPointObserverNode : public ExtensionNode
{
public:
	GridPointObserverNode(const rclcpp::NodeOptions &options) : GridPointObserverNode("", options) {}
	GridPointObserverNode(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("grid_point_observer_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start grid_point_observer_node");
		// get param
		std::string LASER_POSE_TOPIC = param<std::string>("grid_point_observer.topic_name.laser_pose", "ndt_scan_matcher/laser_pose");
		std::string ODOM_TOPIC = param<std::string>("grid_point_observer.topic_name.odom", "/odom");
		std::string IMU_TOPIC = param<std::string>("grid_point_observer.topic_name.imu", "/wit_ros/imu_pose");
		std::string ESTIMATE_POSE_TOPIC = param<std::string>("grid_point_observer.topic_name.estimate_pose", "grid_point_observer/estimate_pose");
		std::string ESTIMATE_TWIST_TOPIC = param<std::string>("grid_point_observer.topic_name.estimate_twist", "grid_point_observer/estimate_twist");
		// frame
		MAP_FRAME = param<std::string>("grid_point_observer.tf_frame.map_frame", "map");
		ODOM_FRAME = param<std::string>("grid_point_observer.tf_frame.odom_frame", "odom");
		ROBOT_FRAME = param<std::string>("grid_point_observer.tf_frame.robot_frame", "base_link");
		// setup
		BROADCAST_PERIOD = param<double>("grid_point_observer.broadcast_period", 0.001);
		ODOM_TF = param<bool>("grid_point_observer.odom_tf_broadcast", true);
		// grid_point_observer
		grid_point_observer_parameter_t gpo_param;
		gpo_param.grid_width = param<double>("grid_point_observer.gpo.grid_width", 0.01);
		gpo_param.min_gain_position = param<double>("grid_point_observer.gpo.min_gain_position", 0.1);
		gpo_param.min_gain_orientation = param<double>("grid_point_observer.gpo.min_gain_orientation", 0.03);
		gpo_param.max_gain_position = param<double>("grid_point_observer.gpo.max_gain_position", 0.4);
		gpo_param.max_gain_orientation = param<double>("grid_point_observer.gpo.max_gain_orientation", 0.1);
		// robotパラメータ
		MIN_VEL = param<double>("grid_point_observer.robot.min_velocity", 0.01);
		MIN_ANGULAR = param<double>("grid_point_observer.robot.min_angular", 0.01);
		gpo_param.max_velocity = param<double>("grid_point_observer.robot.max_velocity", 5.0);
		gpo_param.max_angular = param<double>("grid_point_observer.robot.max_angular", 5.0);
		// init
		gpo_.set_params(gpo_param);
		odom_pose_.position = estimate_pose_.position = {0.0, 0.0, 0.0};
		odom_pose_.orientation = estimate_pose_.orientation = {0.0, 0.0, 0.0, 1.0};
		initialization_ = true;
		// publisher
		estimate_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ESTIMATE_POSE_TOPIC, rclcpp::QoS(10).best_effort());
		estimate_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(ESTIMATE_TWIST_TOPIC, rclcpp::QoS(10).best_effort());
		// subscriber
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(ODOM_TOPIC, rclcpp::QoS(10), std::bind(&GridPointObserverNode::odometry_callback, this, std::placeholders::_1));
		laser_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(LASER_POSE_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&GridPointObserverNode::laser_pose_callback, this, std::placeholders::_1));
		imu_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(IMU_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&GridPointObserverNode::imu_callback, this, std::placeholders::_1));
		// timer
		main_timer_ = this->create_wall_timer(1s * BROADCAST_PERIOD, [&]()
											  {
			// grid point observe
			const auto &[estimate_position, estimate_orientation] = gpo_.update_estimate_pose();
			estimate_pose_.position = estimate_position;
			estimate_pose_.orientation.set_rpy(estimate_orientation.x, estimate_orientation.y, estimate_orientation.z);
			// estimate_pose_.orientation=imu_pose_.orientation;
			estimate_twist_.linear = odom_twist_.linear.get_rotated(estimate_orientation);
			estimate_twist_.angular = odom_twist_.angular;
			// map -> odom tf
			if (ODOM_TF)
			{
				geometry_msgs::msg::TransformStamped transform_stamped;
				transform_stamped.header = make_header(MAP_FRAME, rclcpp::Clock().now());
				transform_stamped.child_frame_id = ODOM_FRAME;
				transform_stamped.transform = make_geometry_transform(odom_pose_);
				broadcaster_.sendTransform(transform_stamped);
			}
			// map -> base_link tf
			if (estimate_pose_.position.has_nan() || estimate_pose_.orientation.has_nan())
			{
				estimate_pose_.position = {0.0, 0.0, 0.0};
				estimate_pose_.orientation = {0.0, 0.0, 0.0, 1.0};
			}

			geometry_msgs::msg::PoseStamped estimate_pose_msg;
			estimate_pose_msg.header = make_header(MAP_FRAME, rclcpp::Clock().now());
			estimate_pose_msg.pose = make_geometry_pose(estimate_pose_);
			geometry_msgs::msg::TwistStamped estimate_twist_msg;
			estimate_twist_msg.header = make_header(ROBOT_FRAME, rclcpp::Clock().now());
			estimate_twist_msg.twist = make_geometry_twist(estimate_twist_);

			estimate_pose_pub_->publish(estimate_pose_msg);
			estimate_twist_pub_->publish(estimate_twist_msg);
			geometry_msgs::msg::TransformStamped transform_stamped;
			transform_stamped.header = make_header(MAP_FRAME, rclcpp::Clock().now());
			transform_stamped.child_frame_id = ROBOT_FRAME;
			transform_stamped.transform = make_geometry_transform(estimate_pose_);
			broadcaster_.sendTransform(transform_stamped); });
	}

	// callback
	void odometry_callback(const nav_msgs::msg::Odometry::ConstPtr msg)
	{
		odom_twist_ = make_twist(*msg);
		odom_pose_ = make_pose(*msg);
		gpo_.set_deadreckoning(odom_pose_.position, odom_pose_.orientation.get_rpy(), odom_twist_);
	}

	void imu_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg)
	{
		imu_pose_ = make_pose(msg->pose);
		if(initialization_)
		{
			init_imu_pose_ = imu_pose_;
			initialization_ = false;
		}
		Vector3d rpy= imu_pose_.orientation.get_rpy()-init_imu_pose_.orientation.get_rpy();
		imu_pose_.orientation.set_rpy(rpy.x,rpy.y,rpy.z);
		gpo_.set_deadreckoning(imu_pose_.position, imu_pose_.orientation.get_rpy(), odom_twist_);//てすと
	}

	void laser_pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg)
	{
		Pose3d laser_pose = make_pose(msg->pose);
		laser_pose.position += estimate_twist_.linear * (double)(this->get_clock()->now().seconds() - rclcpp::Time(msg->header.stamp).seconds());
		gpo_.set_starreckoning(laser_pose.position, laser_pose.orientation.get_rpy());
	}

private:
	bool initialization_;
	// param
	std::string MAP_FRAME;
	std::string ROBOT_FRAME;
	std::string ODOM_FRAME;
	double BROADCAST_PERIOD;
	double MIN_VEL;
	double MIN_ANGULAR;
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
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr laser_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub_;
	// publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimate_twist_pub_;
	// filter
	GridPointObserver gpo_;
	// pose
	Pose3d odom_pose_;
	Pose3d imu_pose_;
	Pose3d init_imu_pose_;
	Pose3d estimate_pose_;
	// vel
	Twistd odom_twist_;
	Twistd estimate_twist_;
};