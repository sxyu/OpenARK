//
// Created by lucas on 1/28/18.
//

#ifndef OPENARK_TYPES_H
#define OPENARK_TYPES_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

namespace ark{

    typedef pcl::PointXYZRGB PointType;

    //container for storing syncronized imu measurements
    struct ImuPair{
        double timestamp;
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
    };

    //container for storing measurement from a single sensor
    enum class ImuMeasurementType {gyro,accel};
    struct ImuMeasurement{
      double timestamp;
      Eigen::Vector3d data;
      ImuMeasurementType type;
      bool operator<(const ImuMeasurement& right) const
        {
          return timestamp>right.timestamp;
        }
    };

    class RGBDFrame {
    public:
        cv::Mat mTcw;
        cv::Mat imRGB;
        cv::Mat imDepth;
        int frameId;
        RGBDFrame(){
            mTcw = cv::Mat::eye(4,4,CV_32FC1);
            frameId = -1;
        }
        RGBDFrame(const RGBDFrame& frame)
        {
            frame.mTcw.copyTo(mTcw);
            frame.imRGB.copyTo(imRGB);
            frame.imDepth.copyTo(imDepth);
            frameId = frame.frameId;
        }
    };

    class MultiCameraFrame {
    public:
        std::vector<cv::Mat> images;
        std::vector<cv::Mat> vecTcw;
        int frameId;
        MultiCameraFrame(){
            frameId = -1;
        }
        MultiCameraFrame(const MultiCameraFrame& frame):
        images(frame.images.size()),
        vecTcw(frame.vecTcw.size()),
        frameId(frame.frameId)
        {   
            for(size_t i=0; i<frame.vecTcw.size(); i++){
                frame.vecTcw[i].copyTo(vecTcw[i]);
            }
            for(size_t i=0; i<frame.images.size(); i++){
                frame.images[i].copyTo(images[i]);
            }
        }
    };
}


#endif //OPENARK_TYPES_H