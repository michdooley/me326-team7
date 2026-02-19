#ifndef TIDYBOT_GPD_GRASP_PLANNER_NODE_HPP_
#define TIDYBOT_GPD_GRASP_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tidybot_msgs/srv/plan_grasp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gpd/util/config_file.h>
#include <gpd/grasp_detector.h>
#include <gpd/candidate/hand.h>

/**
 * @brief ROS2 gripper for converting depth images + bounding boxes to 3D object positions.
 *
 * This node implements the /plan_grasp service, which takes a 3D object position
 * and returns viable grasp poses using the GPD (Grasp Pose Detection) library.
 *
 * Services:
 *   /plan_grasp (tidybot_msgs/PlanGrasp) - plan a grasp for an object
 *
 * Parameters:
 *   config_file (string): path to GPD config file (required)
 *   approach_offset (double): pre-grasp offset in meters (default 0.10)
 *   default_arm (string): arm to use for grasp planning (default "right")
 *   point_cloud_timeout (double): timeout for waiting on point cloud (default 2.0s)
 */
class GraspPlannerNode : public rclcpp::Node {
 public:
  explicit GraspPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GraspPlannerNode();

 private:
  // Service callback
  void plan_grasp_callback(const std::shared_ptr<tidybot_msgs::srv::PlanGrasp::Request> request,
                            std::shared_ptr<tidybot_msgs::srv::PlanGrasp::Response> response);

  // Subscribers
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  // Helper methods
  pcl::PointCloud<pcl::PointXYZ> point_cloud2_to_pcl(const sensor_msgs::msg::PointCloud2& cloud_msg);
  geometry_msgs::msg::Pose hand_to_pose(const gpd::candidate::Hand& hand);

  // ROS2 service, subscribers, publishers
  rclcpp::Service<tidybot_msgs::srv::PlanGrasp>::SharedPtr grasp_service_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // TF2 for frame transformations
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // GPD components
  std::unique_ptr<gpd::GraspDetector> grasp_detector_;

  // Stored point cloud (latest received)
  pcl::PointCloud<pcl::PointXYZ> latest_cloud_;
  rclcpp::Time latest_cloud_time_;
  mutable std::mutex cloud_mutex_;

  // Parameters
  std::string config_file_;
  double approach_offset_;
  std::string default_arm_;
  double point_cloud_timeout_;
};

#endif  // TIDYBOT_GPD_GRASP_PLANNER_NODE_HPP_
