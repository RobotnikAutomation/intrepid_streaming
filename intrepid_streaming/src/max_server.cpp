#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <intrepid_streaming_msgs/MAXStream.h>

#include <lz4.h>

geometry_msgs::Pose lidar_pose_, camera_pose_;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::NavSatFix, nav_msgs::Odometry> testSyncPolicy;
// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CameraInfo, sensor_msgs::Imu> testSyncPolicy;

ros::Publisher compressed_publisher_;


inline int16_t to_cm_and_clamp(const float& v)
{
  float in_cm = v * 100;

  if (in_cm > 32767) return static_cast<int16_t>(32767);
  if (in_cm < -32768) return static_cast<int16_t>(-32768);

  return static_cast<int16_t>(in_cm);
}


sensor_msgs::PointCloud2 compress_lidar_msg(const sensor_msgs::PointCloud2& input)
{
  sensor_msgs::PointCloud2 compressed, output;

  output.header = input.header;

  sensor_msgs::PointCloud2Modifier modifier(output);

  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::INT16,
                                   "y", 1, sensor_msgs::PointField::INT16,
                                   "z", 1, sensor_msgs::PointField::INT16,
                                   "intensity", 1, sensor_msgs::PointField::FLOAT32);

  modifier.resize(input.height * input.width);

  sensor_msgs::PointCloud2ConstIterator<float> input_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> input_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> input_z(input, "z");
  sensor_msgs::PointCloud2ConstIterator<float> input_intensity(input, "intensity");

  sensor_msgs::PointCloud2Iterator<int16_t> output_x(output, "x");
  sensor_msgs::PointCloud2Iterator<int16_t> output_y(output, "y");
  sensor_msgs::PointCloud2Iterator<int16_t> output_z(output, "z");
  sensor_msgs::PointCloud2Iterator<float> output_intensity(output, "intensity");

  while(input_x != input_x.end())
  {
    *output_x = to_cm_and_clamp(*input_x);
    *output_y = to_cm_and_clamp(*input_y);
    *output_z = to_cm_and_clamp(*input_z);
    *output_intensity = *input_intensity;

    ++input_x;
    ++input_y;
    ++input_z;
    ++input_intensity;

    ++output_x;
    ++output_y;
    ++output_z;
    ++output_intensity;
  }

  size_t prev_size = output.data.size();

  std::vector<unsigned char> output_compressed(output.data.size());

  auto rv = LZ4_compress_default((const char*)output.data.data(), (char * ) output_compressed.data(), output.data.size(), output_compressed.capacity());

  if(rv < 1) ROS_ERROR_THROTTLE(2,"Something went wrong! %d", rv);
  else output_compressed.resize(rv);

  compressed.header = output.header;
  compressed.height = output.height;
  compressed.width = output.width;
  compressed.fields = output.fields;
  compressed.is_bigendian = output.is_bigendian;
  compressed.point_step = output.point_step;
  compressed.row_step = output.row_step;
  compressed.is_dense = output.is_dense;
  compressed.data = output_compressed; //lz4 compression

  return compressed;
}

// Callback function
void callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const sensor_msgs::ImageConstPtr &rgb_image, const sensor_msgs::ImageConstPtr &depth_image,const sensor_msgs::ImageConstPtr &thermal_image, const sensor_msgs::CameraInfoConstPtr &depth_camera_info, const sensor_msgs::CameraInfoConstPtr &thermal_camera_info, const sensor_msgs::NavSatFixConstPtr &ugv_position, const nav_msgs::OdometryConstPtr &ugv_odometry)  // The callback contains multiple messages
// void callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const sensor_msgs::CompressedImageConstPtr &rgb_image, const sensor_msgs::CompressedImageConstPtr &depth_image,const sensor_msgs::CameraInfoConstPtr &camera_info, const sensor_msgs::ImuConstPtr &imu_data)  // The callback contains multiple messages
{
  ROS_INFO("GOT ALL SYNCED DATA. STREAMING");
  intrepid_streaming_msgs::MAXStream msg;
  msg.header.stamp = ros::Time::now();
  //msg.lidar = compress_lidar_msg(*pointcloud);
  msg.lidar = *pointcloud;
  std_msgs::String lidar_type;
  lidar_type.data = "ouster16";
  msg.lidar_type = lidar_type;
  msg.image = *rgb_image;
  msg.depth = *depth_image;
  msg.thermal = *thermal_image;
  msg.camera_info_depth = *depth_camera_info;
  msg.camera_info_thermal = *thermal_camera_info;
  msg.ugv_position = *ugv_position;
  msg.ugv_orientation = ugv_odometry->pose.pose.orientation;
  // Slight angle on ouster, may need to add that?
  msg.lidar_pose = ugv_odometry->pose.pose;
  //msg.lidar_pose = lidar_pose_;
  msg.depth_camera_pose = ugv_odometry->pose.pose;
  msg.thermal_camera_pose = ugv_odometry->pose.pose;
  //msg.camera_pose = camera_pose_;

  compressed_publisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrepid_max_streaming_server");
  // subscribe to lidar, images, camera info, etc and publish
  ros::NodeHandle nh;
  compressed_publisher_ = nh.advertise<intrepid_streaming_msgs::MAXStream>("compressed_max_stream", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_subscriber(nh, "/max/lidar", 1);
  message_filters::Subscriber<sensor_msgs::Image> rgb_subscriber(nh, "/max/rgb", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_subscriber(nh, "/max/depth", 1);
  message_filters::Subscriber<sensor_msgs::Image> thermal_subscriber(nh, "/max/thermal", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_depth_subscriber(nh, "/max/depth_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_thermal_subscriber(nh, "/max/thermal_info", 1);
  message_filters::Subscriber<sensor_msgs::NavSatFix> ugv_gps_subscriber(nh, "/max/global", 1);
  message_filters::Subscriber<nav_msgs::Odometry> ugv_odom_subscriber(nh, "/max/local", 1);
  
  message_filters::Synchronizer<testSyncPolicy> sync(testSyncPolicy(100), lidar_subscriber, rgb_subscriber, depth_subscriber, thermal_subscriber, camera_info_depth_subscriber, camera_info_thermal_subscriber, ugv_gps_subscriber, ugv_odom_subscriber);
//  message_filters::Synchronizer<testSyncPolicy> sync(testSyncPolicy(10), lidar_subscriber, rgb_subscriber, depth_subscriber, camera_info_subscriber, imu_subscriber);
  // sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
 sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8));

/*
  tf::TransformListener listener;
  tf::StampedTransform lidar_transform, camera_transform;
  tf::Quaternion quaternion;
  bool transforms_received;

   while (!transforms_received)
   {
     transforms_received = true;

     if(listener.waitForTransform("/robot_top_3d_laser_link", "/robot_base_link", ros::Time(0), ros::Duration(1.0)) == false)
     {
       ROS_WARN_STREAM("Waiting for transform from robot_top_3d_laser_link to robot_base_link");
       transforms_received = false;
     }

     if(listener.waitForTransform("/camera_ugv_link", "/robot_base_link", ros::Time(0), ros::Duration(1.0)) == false)
     {
       ROS_WARN_STREAM("Waiting for transform from camera_link to robot_base_link");
       transforms_received = false;
     }
   }

   //listener.lookupTransform("/robot_top_3d_laser_link", "/robot_base_link",
   //ros::Time(0), lidar_transform);
   lidar_pose_.position.x = 0;
   lidar_pose_.position.y = 0;
   lidar_pose_.position.z = 0;
   lidar_pose_.orientation.x = 0;
   lidar_pose_.orientation.y = 0;
   lidar_pose_.orientation.z = 0;
   lidar_pose_.orientation.w = 1;
   
   

   listener.lookupTransform("/camera_ugv_link", "/robot_base_link",
   ros::Time(0), camera_transform);
   camera_pose_.position.x = camera_transform.getOrigin().x();
   camera_pose_.position.y = camera_transform.getOrigin().y();
   camera_pose_.position.z = camera_transform.getOrigin().z();
   quaternion = camera_transform.getRotation();
   tf::quaternionTFToMsg(quaternion, camera_pose_.orientation);
   
   camera_pose_.position.x = 0;
   camera_pose_.position.y = 0;
   camera_pose_.position.z = 0;
   camera_pose_.orientation.x = 0;
   camera_pose_.orientation.y = 0;
   camera_pose_.orientation.z = 0;
   camera_pose_.orientation.w = 1;
   */
   
   
   ros::spin();

   return 0;
}
