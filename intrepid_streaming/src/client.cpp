#include <ros/ros.h>

#include <intrepid_streaming_msgs/UGVStream.h>
#include <intrepid_streaming_msgs/CompressedUGVStream.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <lz4.h>

namespace enc = sensor_msgs::image_encodings;

inline float to_m(const int16_t& v)
{
    return v / 100.0f;
}

ros::Publisher decompressed_publisher_;

sensor_msgs::PointCloud2 decompress_lidar_msg(const sensor_msgs::PointCloud2& msg)
{
  sensor_msgs::PointCloud2 input;
  sensor_msgs::PointCloud2 output;

  input.header = msg.header;
  input.height = msg.height;
  input.width = msg.width;
  input.fields = msg.fields;
  input.is_bigendian = msg.is_bigendian;
  input.point_step = msg.point_step;
  input.row_step = msg.row_step;
  input.is_dense = msg.is_dense;

  output.header = input.header;

  input.data = msg.data;

  size_t prev_size = input.data.size();

  std::vector<unsigned char> uncompressed(input.height*input.width*input.point_step);

 	auto rv = LZ4_decompress_safe((const char*)input.data.data(), (char*) uncompressed.data(), input.data.size(), uncompressed.size());
	if(rv < 1)  ROS_ERROR_THROTTLE(2,"Something went wrong!");
	else uncompressed.resize(rv);

  input.data = uncompressed;

  ROS_INFO_THROTTLE(1, "descomprimiendo");

  sensor_msgs::PointCloud2Modifier modifier(output);

  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "intensity", 1, sensor_msgs::PointField::FLOAT32);
                                  //  "ring", 1, sensor_msgs::PointField::UINT16);

  modifier.resize(input.height * input.width);

  sensor_msgs::PointCloud2ConstIterator<int16_t> input_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<int16_t> input_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<int16_t> input_z(input, "z");
  sensor_msgs::PointCloud2ConstIterator<float> input_intensity(input, "intensity");

  sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
  sensor_msgs::PointCloud2Iterator<float> output_intensity(output, "intensity");

  bool first = true;

  while(input_x != input_x.end())
  {
    *output_x = to_m(*input_x);
    *output_y = to_m(*input_y);
    *output_z = to_m(*input_z);
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

  return output;
}

sensor_msgs::Image decompress_camera_msg(const sensor_msgs::CompressedImage& msg, const int& imdecode_flag)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = msg.header;

  // Decode color/mono image
  try
  {
    cv_ptr->image = cv::imdecode(cv::Mat(msg.data), imdecode_flag);

    // Assign image encoding string
    const size_t split_pos = msg.format.find(';');
    if (split_pos==std::string::npos)
    {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels())
      {
        case 1:
          cv_ptr->encoding = enc::MONO8;
          break;
        case 3:
          cv_ptr->encoding = enc::BGR8;
          break;
        default:
          ROS_ERROR("Unsupported number of channels: %i", cv_ptr->image.channels());
          break;
      }
    } else
    {
      std::string image_encoding = msg.format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if ( enc::isColor(image_encoding))
      {
        std::string compressed_encoding = msg.format.substr(split_pos);
        bool compressed_bgr_image = (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image)
        {
          // if necessary convert colors from bgr to rgb
          if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
        } else
        {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16))
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
        }
      }
    }
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0))
  {
    sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();
    // Publish message to user callback
    return *image;
  }
}

void inputCallback(const intrepid_streaming_msgs::CompressedUGVStream& compressed_input)
{
   intrepid_streaming_msgs::UGVStream decompressed_input;

   // decompress input into decompressed_input
   decompressed_input.lidar = decompress_lidar_msg(compressed_input.lidar);
   decompressed_input.image = decompress_camera_msg(compressed_input.image, cv::IMREAD_COLOR);
   decompressed_input.depth = decompress_camera_msg(compressed_input.depth, cv::IMREAD_GRAYSCALE);
   decompressed_input.camera_info = compressed_input.camera_info;
   decompressed_input.ugv_pose = compressed_input.ugv_pose;
   decompressed_input.lidar_pose = compressed_input.lidar_pose;
   decompressed_input.camera_pose = compressed_input.camera_pose;

   // publish decompressed_input
   decompressed_publisher_.publish(decompressed_input);
}


int main(int argc, char **argv)
{
  ros::NodeHandle nh;

  ros::Subscriber compressed_subscriber = nh.subscribe("compressed_ugv_stream", 1, inputCallback);
  decompressed_publisher_ = nh.advertise<intrepid_streaming_msgs::UGVStream>("ugv_stream", 1);

  return 0;
}
