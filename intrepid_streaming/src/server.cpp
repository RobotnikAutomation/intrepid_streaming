#include <ros/ros.h>

#include <intrepid_streaming_msgs/CompressedUGVStream.h>

int main(int argc, char **argv)
{
  // subscribe to lidar, images, camera info, etc and publish 
  ros::NodeHandle nh;
  ros::Publisher compressed_publisher = nh.advertise<intrepid_streaming_msgs::CompressedUGVStream>("compressed_ugv_stream", 1);
  return 0;
}
