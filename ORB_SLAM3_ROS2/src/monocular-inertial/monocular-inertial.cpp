#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "monocular-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam3 mono-inertial path_to_vocabulary path_to_settings [do_equalize] [enable_visualization] [use_compressed]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    std::string doEqualize = "false";
    if(argc > 3)
    {
        doEqualize = argv[3];
    }

    bool visualization = true;
    if(argc > 4)
    {
        std::string vizArg = argv[4];
        visualization = (vizArg == "true" || vizArg == "1");
    }

    std::string useCompressed = "false";
    if(argc > 5)
    {
        useCompressed = argv[5];
    }

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    auto node = std::make_shared<MonocularInertialNode>(&pSLAM, argv[2], doEqualize, useCompressed);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
