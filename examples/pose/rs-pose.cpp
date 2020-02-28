// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>

const float pi = 3.1415926;


#define IPADDRESS "10.41.43.255"
//#define IPADDRESS "10.255.255.255"
//#define IPADDRESS "127.0.0.1"
#define PORT 4143

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define WSAGetLastError() (errno)
#define SOCKET_ERROR (-1)

struct sockaddr_in si_other;
int s, slen;

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    //cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF, 0, 0, RS2_FORMAT_ANY, 100);
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);


  //struct sockaddr_in si_other;
  s = sizeof(si_other);
  slen = sizeof(si_other);

  //create socket
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
    std::cout << "sendto() failed with error code : " << WSAGetLastError() << std::endl;
    exit(EXIT_FAILURE);
  }
  int broadcastEnable = 1;
  int ret = setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
  std::cout << "broadcast ret: " << ret << std::endl;

  //setup address structure
  std::memset((char *)&si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);
  inet_aton(IPADDRESS, &si_other.sin_addr);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();


	// calculate euler angles from quaterions
        auto pose_rotation = pose_data.rotation;
        auto w = pose_rotation.w;
	auto x = pose_rotation.z;
	auto y = pose_rotation.x;
	auto z = -pose_rotation.y;

	auto pitch = -asin(2.0 * (x*z - w*y)) * 180.0 / pi;
	auto roll = atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / pi;
	auto yaw = atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / pi;


        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            pose_data.translation.y << " " << pose_data.translation.z << " (meters) " <<
	pitch << " " << roll << " " << yaw << std::endl;


	std::stringstream message;
	message << std::to_string(pitch) << " " << std::to_string(roll) << " " << std::to_string(yaw) << " " << std::to_string(pose_data.translation.x) << " " << std::to_string(pose_data.translation.y) << " " << std::to_string(pose_data.translation.z);
      std::cout << "sending " << message.str() << std::endl;
      if (sendto(s, message.str().c_str(), message.str().size(), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
      {
        std::cout << "sendto() failed with error code : " << WSAGetLastError() << std::endl;
      }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
