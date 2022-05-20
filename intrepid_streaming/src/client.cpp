#include <ros/ros.h>

#include <intrepid_streaming_msgs/UGVStream.h>
#include <intrepid_streaming_msgs/CompressedUGVStream.h>

void inputCallback(const intrepid_streaming_msgs::CompressedUGVStream& compressed_input)
{
   intrepid_streaming_msgs::UGVStream decompressed_input;

   // decompress input into decompressed_input

   // publish decompressed_input
}


int main(int argc, char **argv)
{
  ros::NodeHandle nh;
  
  ros::Subscriber compressed_subscriber = nh.subscribe("compressed_ugv_stream", 1, inputCallback);
  ros::Publisher decompressed_publisher = nh.advertise<intrepid_streaming_msgs::UGVStream>("ugv_stream", 1);
  
  return 0;
}
