# 3D position of a detected object in the world
std_msgs/Header header
string class_name                       # what object this is
geometry_msgs/PointStamped position     # 3D position in base_link frame
float32 confidence                      # detection confidence
