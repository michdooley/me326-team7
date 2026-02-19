#include "tidybot_gpd/grasp_planner_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_set.h>

#include <memory>
#include <chrono>

GraspPlannerNode::GraspPlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("grasp_planner_node", options) {
  // Declare parameters
  declare_parameter<std::string>("config_file", "");
  declare_parameter<double>("approach_offset", 0.10);
  declare_parameter<std::string>("default_arm", "right");
  declare_parameter<double>("point_cloud_timeout", 2.0);

  // Get parameters
  config_file_ = get_parameter("config_file").as_string();
  approach_offset_ = get_parameter("approach_offset").as_double();
  default_arm_ = get_parameter("default_arm").as_string();
  point_cloud_timeout_ = get_parameter("point_cloud_timeout").as_double();

  RCLCPP_INFO(get_logger(), "GraspPlannerNode initializing...");
  RCLCPP_INFO(get_logger(), "  config_file: %s", config_file_.c_str());
  RCLCPP_INFO(get_logger(), "  approach_offset: %.3f m", approach_offset_);
  RCLCPP_INFO(get_logger(), "  default_arm: %s", default_arm_.c_str());

  // Initialize TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load GPD config and initialize detector
  if (config_file_.empty()) {
    RCLCPP_ERROR(get_logger(),
                 "config_file parameter not set! Please provide the path to GPD config file.");
    return;
  }

  try {
    grasp_detector_ = std::make_unique<gpd::GraspDetector>(config_file_);
    RCLCPP_INFO(get_logger(), "GPD detector initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize GPD detector: %s", e.what());
    return;
  }

  // Create service
  grasp_service_ = create_service<tidybot_msgs::srv::PlanGrasp>(
      "/plan_grasp", std::bind(&GraspPlannerNode::plan_grasp_callback, this,
                               std::placeholders::_1, std::placeholders::_2));

  // Subscribe to point cloud
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/cloud", rclcpp::SensorDataQoS(),
      std::bind(&GraspPlannerNode::point_cloud_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "GraspPlannerNode ready. Waiting for point clouds on /camera/depth/cloud");
}

GraspPlannerNode::~GraspPlannerNode() = default;

void GraspPlannerNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = point_cloud2_to_pcl(*cloud_msg);
  latest_cloud_time_ = cloud_msg->header.stamp;
  RCLCPP_DEBUG(get_logger(), "Received point cloud with %ld points", latest_cloud_.size());
}

void GraspPlannerNode::plan_grasp_callback(
    const std::shared_ptr<tidybot_msgs::srv::PlanGrasp::Request> request,
    std::shared_ptr<tidybot_msgs::srv::PlanGrasp::Response> response) {
  RCLCPP_INFO(get_logger(), "Planning grasp for object class: %s", request->object_class.c_str());

  if (!grasp_detector_) {
    response->success = false;
    response->message = "GPD detector not initialized. Check config_file parameter.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Get latest point cloud
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (latest_cloud_.empty()) {
    response->success = false;
    response->message = "No point cloud received yet";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Check if point cloud is fresh
  auto time_diff = (now() - latest_cloud_time_).seconds();
  if (time_diff > point_cloud_timeout_) {
    response->success = false;
    response->message = "Point cloud is stale (received " + std::to_string(time_diff) + " seconds ago)";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  try {
    // Run GPD detector on the point cloud
    RCLCPP_INFO(get_logger(), "Running GPD on point cloud with %ld points...", latest_cloud_.size());
    // Convert pcl::PointCloud<pcl::PointXYZ> to gpd::util::PointCloudRGB
    using PointXYZRGBA = pcl::PointXYZRGBA;
    pcl::PointCloud<PointXYZRGBA>::Ptr cloud_rgb(new pcl::PointCloud<PointXYZRGBA>());
    cloud_rgb->width = static_cast<uint32_t>(latest_cloud_.size());
    cloud_rgb->height = 1;
    cloud_rgb->is_dense = false;
    cloud_rgb->points.resize(latest_cloud_.size());
    for (size_t i = 0; i < latest_cloud_.size(); ++i) {
      const auto &p = latest_cloud_.points[i];
      PointXYZRGBA q;
      q.x = p.x; q.y = p.y; q.z = p.z;
      q.r = q.g = q.b = 0;
      q.a = 255;
      cloud_rgb->points[i] = q;
    }

    // camera_source: single camera observing all points
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Constant(1, (int)cloud_rgb->points.size(), 1);
    Eigen::Matrix3Xd view_points = Eigen::Matrix3Xd::Zero(3, 1);

    gpd::util::Cloud gpd_cloud(cloud_rgb, camera_source, view_points);

    std::vector<std::unique_ptr<gpd::candidate::Hand>> hands = grasp_detector_->detectGrasps(gpd_cloud);

    if (hands.empty()) {
      response->success = false;
      response->message = "No grasps found in point cloud";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    // Get the best grasp (highest score)
    size_t best_idx = 0;
    double best_score = -1.0;
    for (size_t i = 0; i < hands.size(); ++i) {
      if (!hands[i]) continue;
      double score = hands[i]->getScore();
      if (score > best_score) {
        best_score = score;
        best_idx = i;
      }
    }

    RCLCPP_INFO(get_logger(), "Found %ld grasp candidates, best score: %.4f", hands.size(), best_score);

    // Convert best grasp to ROS message
    response->grasp_pose = hand_to_pose(*hands[best_idx]);

    // Create pre-grasp pose by offsetting along approach direction
    // For parallel-jaw grippers, approach direction is typically along Z (down)
    response->pre_grasp_pose = response->grasp_pose;
    response->pre_grasp_pose.position.z += approach_offset_;

    // Set other response fields
    response->arm_used = request->arm_name == "auto" ? default_arm_ : request->arm_name;
    response->grasp_width = 0.08;  // Default gripper width (adjust to your gripper)
    response->success = true;
    response->message = "Grasp planned successfully";

    RCLCPP_INFO(get_logger(),
                "Grasp pose: position=(%.3f, %.3f, %.3f), orientation=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                response->grasp_pose.position.x, response->grasp_pose.position.y,
                response->grasp_pose.position.z, response->grasp_pose.orientation.w,
                response->grasp_pose.orientation.x, response->grasp_pose.orientation.y,
                response->grasp_pose.orientation.z);

  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Exception during grasp planning: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

pcl::PointCloud<pcl::PointXYZ> GraspPlannerNode::point_cloud2_to_pcl(
    const sensor_msgs::msg::PointCloud2& cloud_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  return cloud;
}

geometry_msgs::msg::Pose GraspPlannerNode::hand_to_pose(const gpd::candidate::Hand& hand) {
  geometry_msgs::msg::Pose pose;

  // Get grasp position
  Eigen::Vector3d position = hand.getPosition();
  pose.position.x = position(0);
  pose.position.y = position(1);
  pose.position.z = position(2);

  // Get grasp orientation (rotation matrix to quaternion)
  Eigen::Matrix3d rotation = hand.getOrientation();
  Eigen::Quaterniond quat(rotation);

  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  return pose;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
