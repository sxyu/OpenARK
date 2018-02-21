#include <utility>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "HandFeatureExtractor.h"
#include "HandClassifier.h"
#include "core.h"

namespace {
    template <class T>
    inline void readBinary(std::ifstream & ifs, T * val) {
        ifs.read(reinterpret_cast<char *>(val), sizeof(*val));
    }

    void readDM(cv::Mat & m, std::string path) {
        std::ifstream ifs(path, std::ios::binary | std::ios::in);

        ushort wid, hi;
        readBinary(ifs, &hi);
        readBinary(ifs, &wid);

        m = cv::Mat::zeros(hi, wid, CV_32FC3);

        int zr = 0;

        for (int i = 0; i < hi; ++i) {
            cv::Vec3f * ptr = m.ptr<cv::Vec3f>(i);
            for (int j = 0; j < wid; ++j) {
                if (zr) --zr;
                else {
                    if (!ifs) break;
                    float x; readBinary(ifs, &x);
                    if (x <= 1) {
                        ptr[j][0] = x;
                        readBinary(ifs, &ptr[j][1]);
                        readBinary(ifs, &ptr[j][2]);
                    }
                    else {
                        zr = (int)x - 2;
                    }
                }
            }
        }
    }
}

namespace classifier {
    namespace features {

        // default image extension
        const std::string IMG_EXT = ".dm";

        std::vector<double> extractHandFeatures(const cv::Mat & depth_original) {
            cv::Mat depth;
            if (depth_original.type() != CV_32FC3) {
                depth_original.convertTo(depth, CV_32FC3, 1.0 / 256);
            }
            else {
                depth = depth_original;
            }

            ark::ObjectParams params;
            params.handUseSVM = false;
            params.handRequireEdgeConnected = false;

            ark::Hand hand(depth, &params);
            return ark::classifier::HandClassifier::extractShapeFeatures(hand, hand.getDepthMap());
        }

        std::vector<double> extractHandFeatures(std::string testCaseName, std::string dataDir, std::string depthPath) {
            if (dataDir[dataDir.size() - 1] != '/' && dataDir[dataDir.size() - 1] != '\\') {
                dataDir += boost::filesystem::path::preferred_separator;
            }

            // read depth map
            cv::Mat depth;
            if (IMG_EXT == ".dm")
                readDM(depth, dataDir + depthPath + testCaseName + IMG_EXT);
            else
                depth = cv::imread(dataDir + depthPath + testCaseName + IMG_EXT);
            return extractHandFeatures(depth);
        }
    }
}
