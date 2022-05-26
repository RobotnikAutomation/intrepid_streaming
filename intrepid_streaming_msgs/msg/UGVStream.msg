std_msgs/Header header

# pointcloud coming from lidar synchronized with camera images
sensor_msgs/PointCloud2 lidar

# rgb image synchronized with depth image and lidar
sensor_msgs/Image image

# depth image synchronized with rgb image and lidar
sensor_msgs/Image depth

# calibration info from camera
sensor_msgs/CameraInfo camera_info

# ugv pose in world coordinates
sensor_msgs/NavSatFix ugv_pose

# lidar pose with respect to ugv base_link
geometry_msgs/Pose lidar_pose

# camera pose with respect to ugv base_link
geometry_msgs/Pose camera_pose
