std_msgs/Header header

# pointcloud coming from lidar synchronized with camera images
sensor_msgs/PointCloud2 lidar

# Lidar sensor type
std_msgs/String lidar_type

# rgb image synchronized with depth image and lidar
sensor_msgs/CompressedImage image

# depth image synchronized with rgb image and lidar
sensor_msgs/CompressedImage depth

# thermal image synchronized
sensor_msgs/CompressedImage thermal

# calibration info from depth camera
sensor_msgs/CameraInfo camera_info_depth

# calibration info from thermal camera
sensor_msgs/CameraInfo camera_info_thermal

# GPS message
sensor_msgs/NavSatFix ugv_position

# Orientation message
geometry_msgs/Quaternion ugv_orientation

# ugv pose in world coordinates
sensor_msgs/NavSatFix ugv_pose

# lidar pose with respect to ugv base_link
geometry_msgs/Pose lidar_pose

# depth camera pose with respect to ugv base_link
geometry_msgs/Pose depth_camera_pose

# thermal camera pose with respect to ugv base_link
geometry_msgs/Pose thermal_camera_pose
