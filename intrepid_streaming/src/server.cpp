#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <intrepid_streaming_msgs/CompressedUGVStream.h>

#include <lz4.h>

geometry_msgs::Pose lidar_pose_, camera_pose_;

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CameraInfo, sensor_msgs::NavSatFix> testSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CameraInfo> testSyncPolicy;

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
// void callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const sensor_msgs::CompressedImageConstPtr &rgb_image, const sensor_msgs::CompressedImageConstPtr &depth_image,const sensor_msgs::CameraInfoConstPtr &camera_info, const sensor_msgs::NavSatFixConstPtr &ugv_pose)  // The callback contains multiple messages
void callback(const sensor_msgs::PointCloud2ConstPtr &pointcloud, const sensor_msgs::CompressedImageConstPtr &rgb_image, const sensor_msgs::CompressedImageConstPtr &depth_image,const sensor_msgs::CameraInfoConstPtr &camera_info)  // The callback contains multiple messages
{
  intrepid_streaming_msgs::CompressedUGVStream msg;
  msg.header.stamp = ros::Time::now();
  msg.lidar = compress_lidar_msg(*pointcloud);
  msg.image = *rgb_image;
  msg.depth = *depth_image;
  msg.camera_info = *camera_info;
  // msg.ugv_pose = *ugv_pose;
  msg.lidar_pose = lidar_pose_;
  msg.camera_pose = camera_pose_;

  compressed_publisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrepid_streaming_server");
  // subscribe to lidar, images, camera info, etc and publish
  ros::NodeHandle nh;
  compressed_publisher_ = nh.advertise<intrepid_streaming_msgs::CompressedUGVStream>("compressed_ugv_stream", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_subscriber(nh, "/robot/top_3d_laser/points/filtered", 1);
  message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_subscriber(nh, "/robot/camera_ugv/color/image_raw/compressed", 1);
  message_filters::Subscriber<sensor_msgs::CompressedImage> depth_subscriber(nh, "/robot/camera_ugv/aligned_depth_to_color/image_raw/compressedDepth", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subscriber(nh, "/robot/camera_ugv/color/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::NavSatFix> ugv_pose_subscriber(nh, "/robot/gps/filtered", 1);

  // message_filters::Synchronizer<testSyncPolicy> sync(testSyncPolicy(10), lidar_subscriber, rgb_subscriber, depth_subscriber, camera_info_subscriber, ugv_pose_subscriber);
 message_filters::Synchronizer<testSyncPolicy> sync(testSyncPolicy(10), lidar_subscriber, rgb_subscriber, depth_subscriber, camera_info_subscriber);
  // sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));
 sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

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

   listener.lookupTransform("/robot_top_3d_laser_link", "/robot_base_link",
   ros::Time(0), lidar_transform);
   lidar_pose_.position.x = lidar_transform.getOrigin().x();
   lidar_pose_.position.y = lidar_transform.getOrigin().y();
   lidar_pose_.position.z = lidar_transform.getOrigin().z();
   quaternion = lidar_transform.getRotation();
   tf::quaternionTFToMsg(quaternion, lidar_pose_.orientation);

   listener.lookupTransform("/camera_ugv_link", "/robot_base_link",
   ros::Time(0), camera_transform);
   camera_pose_.position.x = camera_transform.getOrigin().x();
   camera_pose_.position.y = camera_transform.getOrigin().y();
   camera_pose_.position.z = camera_transform.getOrigin().z();
   quaternion = camera_transform.getRotation();
   tf::quaternionTFToMsg(quaternion, camera_pose_.orientation);

   ros::spin();

   return 0;
}
