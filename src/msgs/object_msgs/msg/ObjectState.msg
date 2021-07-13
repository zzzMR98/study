# BoundingBox represents a oriented bounding box.
geometry_msgs/Pose pose
sensor_msgs/NavSatFix gps
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
float32 distance
float32 velocity
float32  warning_status
string label
