#include <ros/ros.h>

#include <limits>
#include <string>
#include <vector>

#include <intrepid_streaming_msgs/UGVStream.h>
#include <intrepid_streaming_msgs/CompressedUGVStream.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <lz4.h>

namespace enc = sensor_msgs::image_encodings;

// Compression formats
enum compressionFormat
{
  UNDEFINED = -1, INV_DEPTH
};

// Compression configuration
struct ConfigHeader
{
  // compression format
  compressionFormat format;
  // quantization parameters (used in depth image compression)
  float depthParam[2];
};

inline float to_m(const int16_t& v)
{
    return v / 100.0f;
}

ros::Publisher decompressed_publisher_;
ros::Publisher lidar_publisher_;
ros::Publisher rgb_publisher_;
ros::Publisher depth_publisher_;

int *buffer_;
int *pBuffer_;
int word_;
int nibblesWritten_;

int DecodeVLE() {
  unsigned int nibble;
  int value = 0, bits = 29;
  do {
    if (!nibblesWritten_) {
      word_ = *pBuffer_++;  // load word
      nibblesWritten_ = 8;
    }
    nibble = word_ & 0xf0000000;
    value |= (nibble << 1) >> bits;
    word_ <<= 4;
    nibblesWritten_--;
    bits -= 3;
  } while (nibble & 0x80000000);
  return value;
}

void DecompressRVL(const unsigned char* input, unsigned short* output,
                             int numPixels) {
  buffer_ = pBuffer_ = const_cast<int*>(reinterpret_cast<const int*>(input));
  nibblesWritten_ = 0;
  unsigned short current, previous = 0;
  int numPixelsToDecode = numPixels;
  while (numPixelsToDecode) {
    int zeros = DecodeVLE();  // number of zeros
    numPixelsToDecode -= zeros;
    for (; zeros; zeros--) *output++ = 0;
    int nonzeros = DecodeVLE();  // number of nonzeros
    numPixelsToDecode -= nonzeros;
    for (; nonzeros; nonzeros--) {
      int positive = DecodeVLE();  // nonzero value
      int delta = (positive >> 1) ^ -(positive & 1);
      current = previous + delta;
      *output++ = current;
      previous = current;
    }
  }
}

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

sensor_msgs::Image decompress_rgb_image_msg(const sensor_msgs::CompressedImage& msg, const int& imdecode_flag)
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

sensor_msgs::Image decodeCompressedDepthImage(const sensor_msgs::CompressedImage& message)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message.header;

  // Assign image encoding
  const size_t split_pos = message.format.find(';');
  const std::string image_encoding = message.format.substr(0, split_pos);
  std::string compression_format;
  // Older version of compressed_depth_image_transport supports only png.
  if (split_pos == std::string::npos) {
    compression_format = "png";
  } else {
    std::string format = message.format.substr(split_pos);
    if (format.find("compressedDepth png") != std::string::npos) {
      compression_format = "png";
    } else if (format.find("compressedDepth rvl") != std::string::npos) {
      compression_format = "rvl";
    } else if (format.find("compressedDepth") != std::string::npos && format.find("compressedDepth ") == std::string::npos) {
      compression_format = "png";
    } else {
      ROS_ERROR("Unsupported image format: %s", message.format.c_str());
      return sensor_msgs::Image();
    }
  }

  cv_ptr->encoding = image_encoding;

  // Decode message data
  if (message.data.size() > sizeof(ConfigHeader))
  {

    // Read compression type from stream
    ConfigHeader compressionConfig;
    memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

    // Get compressed image data
    const std::vector<uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

    // Depth map decoding
    float depthQuantA, depthQuantB;

    // Read quantization parameters
    depthQuantA = compressionConfig.depthParam[0];
    depthQuantB = compressionConfig.depthParam[1];

    if (enc::bitDepth(image_encoding) == 32)
    {
      cv::Mat decompressed;
      if (compression_format == "png") {
        try
        {
          // Decode image data
          decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
        }
        catch (cv::Exception& e)
        {
          ROS_ERROR("%s", e.what());
          return sensor_msgs::Image();
        }
      } else if (compression_format == "rvl") {
        const unsigned char *buffer = imageData.data();
        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        decompressed = cv::Mat(rows, cols, CV_16UC1);
        DecompressRVL(&buffer[8], decompressed.ptr<unsigned short>(), cols * rows);
      } else {
        return sensor_msgs::Image();
      }

      size_t rows = decompressed.rows;
      size_t cols = decompressed.cols;

      if ((rows > 0) && (cols > 0))
      {
        cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

        // Depth conversion
        cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>(),
                            itDepthImg_end = cv_ptr->image.end<float>();
        cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
                                          itInvDepthImg_end = decompressed.end<unsigned short>();

        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg)
        {
          // check for NaN & max depth
          if (*itInvDepthImg)
          {
            *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
          }
          else
          {
            *itDepthImg = std::numeric_limits<float>::quiet_NaN();
          }
        }

        // Publish message to user callback
        sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

        return *image;
      }
    }
    else
    {
      // Decode raw image
      if (compression_format == "png") {
        try
        {
          cv_ptr->image = cv::imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);
        }
        catch (cv::Exception& e)
        {
          ROS_ERROR("%s", e.what());
          return sensor_msgs::Image();
        }
      } else if (compression_format == "rvl") {
        const unsigned char *buffer = imageData.data();
        uint32_t cols, rows;
        memcpy(&cols, &buffer[0], 4);
        memcpy(&rows, &buffer[4], 4);
        cv_ptr->image = cv::Mat(rows, cols, CV_16UC1);
        DecompressRVL(&buffer[8], cv_ptr->image.ptr<unsigned short>(), cols * rows);
      } else {
        return sensor_msgs::Image();
      }

      size_t rows = cv_ptr->image.rows;
      size_t cols = cv_ptr->image.cols;

      if ((rows > 0) && (cols > 0))
      {
        // Publish message to user callback
        sensor_msgs::ImagePtr image = cv_ptr->toImageMsg();

        return *image;
      }
    }
  }
  return sensor_msgs::Image();
}

void inputCallback(const intrepid_streaming_msgs::CompressedUGVStream& compressed_input)
{
   intrepid_streaming_msgs::UGVStream decompressed_input;

   // decompress input into decompressed_input
   decompressed_input.lidar = decompress_lidar_msg(compressed_input.lidar);
   decompressed_input.image = decompress_rgb_image_msg(compressed_input.image, cv::IMREAD_COLOR);
   decompressed_input.depth = decodeCompressedDepthImage(compressed_input.depth);
   decompressed_input.camera_info = compressed_input.camera_info;
   decompressed_input.ugv_pose = compressed_input.ugv_pose;
   decompressed_input.lidar_pose = compressed_input.lidar_pose;
   decompressed_input.camera_pose = compressed_input.camera_pose;

   // publish decompressed_input
   decompressed_publisher_.publish(decompressed_input);
   lidar_publisher_.publish(decompressed_input.lidar);
   rgb_publisher_.publish(decompressed_input.image);
   depth_publisher_.publish(decompressed_input.depth);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrepid_streaming_client");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  ros::Subscriber compressed_subscriber = nh.subscribe("compressed_ugv_stream", 1, inputCallback);
  decompressed_publisher_ = pnh.advertise<intrepid_streaming_msgs::UGVStream>("ugv_stream", 1);
  lidar_publisher_ = pnh.advertise<sensor_msgs::PointCloud2>("debug/lidar", 1);
  rgb_publisher_ = pnh.advertise<sensor_msgs::Image>("debug/image", 1);
  depth_publisher_ = pnh.advertise<sensor_msgs::Image>("debug/depth", 1);

  ros::spin();

  return 0;
}
