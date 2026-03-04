/**
 * GPD ROS 2 Service Node
 *
 * Wraps the GPD (Grasp Pose Detection) C++ library as a ROS 2 service.
 * Receives PointCloud2 via /detect_grasps service, runs GPD, returns 6-DOF grasps.
 */

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>

#include "gpd_ros2/srv/detect_grasps.hpp"
#include "gpd_ros2/msg/grasp_config.hpp"
#include "gpd_ros2/msg/grasp_config_list.hpp"

class GpdDetectGraspsServer : public rclcpp::Node {
public:
  GpdDetectGraspsServer() : Node("gpd_detect_grasps_server") {
    // Declare parameters
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<bool>("publish_rviz_markers", true);

    // Get config file path
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'config_file' is required");
      throw std::runtime_error("config_file parameter not set");
    }

    frame_id_ = this->get_parameter("frame_id").as_string();

    // Initialize GPD detector
    RCLCPP_INFO(this->get_logger(), "Loading GPD detector from: %s", config_file.c_str());
    detector_ = std::make_unique<gpd::GraspDetector>(config_file);
    RCLCPP_INFO(this->get_logger(), "GPD detector initialized successfully");

    // Create service
    service_ = this->create_service<gpd_ros2::srv::DetectGrasps>(
      "/detect_grasps",
      std::bind(&GpdDetectGraspsServer::detectGraspsCallback,
                this, std::placeholders::_1, std::placeholders::_2));

    // Optional RViz marker publisher
    if (this->get_parameter("publish_rviz_markers").as_bool()) {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/gpd_grasps_viz", 10);
    }

    RCLCPP_INFO(this->get_logger(), "GPD service ready on /detect_grasps");
  }

private:
  void detectGraspsCallback(
    const std::shared_ptr<gpd_ros2::srv::DetectGrasps::Request> request,
    std::shared_ptr<gpd_ros2::srv::DetectGrasps::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received grasp detection request");

    // 1. Convert ROS PointCloud2 to PCL
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(request->cloud, *pcl_cloud);

    if (pcl_cloud->empty()) {
      response->success = false;
      response->message = "Empty point cloud received";
      RCLCPP_WARN(this->get_logger(), "Empty point cloud");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud has %zu points", pcl_cloud->size());

    // 2. Set up camera view point
    Eigen::Matrix3Xd view_points(3, 1);
    view_points << request->view_point.x,
                   request->view_point.y,
                   request->view_point.z;

    // 3. Camera source: all points from camera 0
    Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, pcl_cloud->size());

    // 4. Create GPD Cloud object
    gpd::util::Cloud gpd_cloud(pcl_cloud, camera_source, view_points);

    // 5. If workspace provided, set sample indices within that workspace
    if (request->workspace.size() == 6) {
      RCLCPP_INFO(this->get_logger(), "Using workspace: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        request->workspace[0], request->workspace[1],
        request->workspace[2], request->workspace[3],
        request->workspace[4], request->workspace[5]);
    }

    // 6. Preprocess and detect
    detector_->preprocessPointCloud(gpd_cloud);

    auto grasps = detector_->detectGrasps(gpd_cloud);

    RCLCPP_INFO(this->get_logger(), "GPD detected %zu grasps", grasps.size());

    if (grasps.empty()) {
      response->success = false;
      response->message = "No grasps detected";
      return;
    }

    // 7. Convert to ROS message
    gpd_ros2::msg::GraspConfigList grasp_list;
    grasp_list.header.stamp = this->now();
    grasp_list.header.frame_id = frame_id_;

    for (const auto& grasp : grasps) {
      gpd_ros2::msg::GraspConfig gc;

      // Position
      gc.position.x = grasp->getPosition()(0);
      gc.position.y = grasp->getPosition()(1);
      gc.position.z = grasp->getPosition()(2);

      // Approach (column 0 of orientation matrix)
      gc.approach.x = grasp->getApproach()(0);
      gc.approach.y = grasp->getApproach()(1);
      gc.approach.z = grasp->getApproach()(2);

      // Binormal (column 1)
      gc.binormal.x = grasp->getBinormal()(0);
      gc.binormal.y = grasp->getBinormal()(1);
      gc.binormal.z = grasp->getBinormal()(2);

      // Axis (column 2)
      gc.axis.x = grasp->getAxis()(0);
      gc.axis.y = grasp->getAxis()(1);
      gc.axis.z = grasp->getAxis()(2);

      // Width and score
      gc.width = grasp->getGraspWidth();
      gc.score = grasp->getScore();

      // Sample point
      gc.sample.x = grasp->getSample()(0);
      gc.sample.y = grasp->getSample()(1);
      gc.sample.z = grasp->getSample()(2);

      grasp_list.grasps.push_back(gc);
    }

    response->grasps = grasp_list;
    response->success = true;
    response->message = "Found " + std::to_string(grasps.size()) + " grasps";

    // 8. Publish RViz markers
    if (marker_pub_) {
      publishGraspMarkers(grasps, grasp_list.header);
    }
  }

  void publishGraspMarkers(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& grasps,
    const std_msgs::msg::Header& header)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int id = 0;
    for (const auto& grasp : grasps) {
      Eigen::Vector3d pos = grasp->getPosition();
      Eigen::Vector3d app = grasp->getApproach();
      Eigen::Vector3d bin = grasp->getBinormal();

      // Approach direction arrow (green)
      visualization_msgs::msg::Marker arrow;
      arrow.header = header;
      arrow.ns = "gpd_approach";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = 0.004;
      arrow.scale.y = 0.008;
      arrow.scale.z = 0.008;
      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 0.0;
      arrow.color.a = 0.8;
      arrow.lifetime = rclcpp::Duration::from_seconds(30.0);

      geometry_msgs::msg::Point start, end;
      start.x = pos(0) - app(0) * 0.05;
      start.y = pos(1) - app(1) * 0.05;
      start.z = pos(2) - app(2) * 0.05;
      end.x = pos(0);
      end.y = pos(1);
      end.z = pos(2);
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      marker_array.markers.push_back(arrow);

      // Grasp center sphere (red)
      visualization_msgs::msg::Marker sphere;
      sphere.header = header;
      sphere.ns = "gpd_center";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = pos(0);
      sphere.pose.position.y = pos(1);
      sphere.pose.position.z = pos(2);
      sphere.scale.x = 0.01;
      sphere.scale.y = 0.01;
      sphere.scale.z = 0.01;
      sphere.color.r = 1.0;
      sphere.color.g = 0.0;
      sphere.color.b = 0.0;
      sphere.color.a = 0.8;
      sphere.lifetime = rclcpp::Duration::from_seconds(30.0);
      marker_array.markers.push_back(sphere);

      // Finger lines (blue) showing gripper opening
      visualization_msgs::msg::Marker fingers;
      fingers.header = header;
      fingers.ns = "gpd_fingers";
      fingers.id = id++;
      fingers.type = visualization_msgs::msg::Marker::LINE_LIST;
      fingers.action = visualization_msgs::msg::Marker::ADD;
      fingers.scale.x = 0.003;
      fingers.color.r = 0.0;
      fingers.color.g = 0.0;
      fingers.color.b = 1.0;
      fingers.color.a = 0.8;
      fingers.lifetime = rclcpp::Duration::from_seconds(30.0);

      double hw = grasp->getGraspWidth() / 2.0;
      geometry_msgs::msg::Point f1, f2;
      f1.x = pos(0) + bin(0) * hw;
      f1.y = pos(1) + bin(1) * hw;
      f1.z = pos(2) + bin(2) * hw;
      f2.x = pos(0) - bin(0) * hw;
      f2.y = pos(1) - bin(1) * hw;
      f2.z = pos(2) - bin(2) * hw;
      fingers.points.push_back(f1);
      fingers.points.push_back(f2);
      marker_array.markers.push_back(fingers);
    }

    marker_pub_->publish(marker_array);
  }

  std::unique_ptr<gpd::GraspDetector> detector_;
  rclcpp::Service<gpd_ros2::srv::DetectGrasps>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string frame_id_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpdDetectGraspsServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
