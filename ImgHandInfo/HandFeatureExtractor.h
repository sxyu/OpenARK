#pragma once

#ifndef PI
    #define PI 3.1415926535898
#endif

#include "core.h"

namespace classifier {
    namespace features {
        /** Extract hand-specific features from a given depth map of a hand.
         *  @param [in] depth_map the depth map
         *  @returns vector of features
         */
        std::vector<double> extractHandFeatures(const cv::Mat & depth_map);

        /* Extract hand-specific features from a test case called "test_case_name" (please include leading zeros in name)
         *  in the data directory "data_dir".
         *  @param test_case_name name of the test case
         *  @param data_dir path to the data directory
         *  @param depth_path optionally, a path to the depth image directory (inferred from data directory by default)
         *  @returns vector of features
         */
        std::vector<double> extractHandFeatures(std::string test_case_name, std::string data_dir, std::string depth_path = "depth/");
    }
}