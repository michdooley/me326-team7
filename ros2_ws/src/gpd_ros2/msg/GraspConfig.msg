# A single 6-DOF grasp candidate from GPD
# Position is the center of the grasp (between the two fingers)
geometry_msgs/Point position

# Orientation vectors defining the grasp frame
geometry_msgs/Vector3 approach    # approach direction (toward object)
geometry_msgs/Vector3 binormal    # finger closing direction
geometry_msgs/Vector3 axis        # hand axis (perpendicular to fingers)

# Grasp quality
float64 width                     # opening width between fingers (meters)
float64 score                     # classifier confidence score

# Sample point where this grasp was generated
geometry_msgs/Point sample
