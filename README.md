# intrepid_streaming

This repository includes the server and client sides for the intrepid pointcloud and images streaming using nimbro.

## Dependencies

- [Nimbro](https://github.com/AIS-Bonn/nimbro_network) (ROS transport for high-latency, low-quality networks)

## Installation

1. Clone the repository using:

`git clone git@github.com:RobotnikAutomation/intrepid_streaming.git`

2. Clone the nimbro package in your workspace:

`git clone git@github.com:AIS-Bonn/nimbro_network.git`

## Launching

The complete launch of the intrepid streaming is done as follows.

- First, configure the client ip in the [server launch file](https://github.com/RobotnikAutomation/intrepid_streaming/blob/master/intrepid_streaming/launch/server.launch):

- Launch the server:

`roslaunch intrepid_streaming server.launch`

- Launch the client:

`roslaunch intrepid_streaming client.launch`

## Topics

### Server

#### Publishers

- `/compressed_ugv_stream`: message that includes the compressed lidar and rgb-d streams.

#### Subscribers

- `/robot/top_3d_laser/scan/filtered`: raw lidar stream

- `/camera/color/image_raw/compressed`: compressed rgb_image

- `/camera/aligned_depth_to_color/image_raw/compressedDepth`: compressed depth image

- `/camera/color/camera_info`: camera information

- `/robot/gps/filtered`: ugv global position

### Client

#### Publishers

- `/ugv_stream`: message that includes the decompressed lidar and rgb-d streams.

- `/debug/lidar`: decompressed lidar stream (only for debugging).

- `/debug/image`: decompressed rgb image (only for debugging).

- `/debug/depth`: decompressed depth image (only for debugging).

#### Subscribers

- `/compressed_ugv_stream`: message that includes the compressed lidar and rgb-d streams.
