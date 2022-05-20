std_msgs/Header header
# pointcloud coming from lidar synchronized with camera images, compressed
# using algorithm from Robotnik. As no CompressedPointCloud2 message exists
# the non-compressed message is used, however the data is not usable unless
# it is decompressed first.
sensor_msgs/PointCloud2 lidar
# rgb image synchronized with depth image and lidar, compressed
sensor_msgs/CompressedImage image
# depth image synchronized with rgb image and lidar, compressed
sensor_msgs/CompressedImage depth
# calibration info from camera
sensor_msgs/CameraInfo camera_info
# ugv pose in world coordinates
geometry_msgs/Pose ugv_pose
# lidar pose with respect to ugv base_link
geometry_msgs/Pose lidar_pose
# camera pose with respect to ugv base_link
geometry_msgs/Pose camera_pose
