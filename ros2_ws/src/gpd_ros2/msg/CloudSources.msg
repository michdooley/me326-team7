# Point cloud with camera source information for GPD
sensor_msgs/PointCloud2 cloud
std_msgs/Int64[] camera_source     # per-point camera index
geometry_msgs/Point[] view_points  # camera position(s)
