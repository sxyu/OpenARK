#pragma once
// OpenCV Libraries
#include "stdafx.h"
#include "version.h"

// OpenARK Libraries
#include "DepthCamera.h"

// RealSense SDK
#include <librealsense/rs.hpp>

//Eigen
#include <Eigen/Core>

#include "single_consumer_priority_queue.h"
#include "types.h"

namespace ark {
  class ZR300Camera : public DepthCamera{
  public: 

    struct FrameObject{
      std::shared_ptr<rs::frame> frame;
      bool operator<(const FrameObject& right) const
      {
        return frame->get_timestamp()> right.frame->get_timestamp();
      }

      double operator-(const FrameObject& right) const
      {
        if(frame!=nullptr&&right.frame!=nullptr)
          return ((frame->get_timestamp())-(right.frame->get_timestamp()));
        if(frame==nullptr)
          std::cerr <<"ERROR in FrameObject -, LEFT NULLPTR" <<std::endl;        
        if(right.frame==nullptr)
          std::cerr <<"ERROR in FrameObject -, RIGHT NULLPTR" <<std::endl;
        return 0;
      }

    };

    explicit ZR300Camera();
    ~ZR300Camera()  override;

    bool nextFrame();

    void update();

    void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
                            cv::Mat & amp_map, cv::Mat & flag_map){
    };

    /**
     * Returns the width of the frame in pixels. 
     */
    int getWidth() const {
        return REAL_WID;
    };

    /**
     * Returns the height of the frame in pixels. 
     */
    int getHeight() const {
        return REAL_HI;
    };
    /**
    * Returns true if an RGB image is available from this camera.
    * @return true if an RGB image is available from this camera.
    */
    bool hasRGBImage() const;

    /**
    * Returns true if an infrared (IR) image is available from this camera.
    * @return true if an infrared (IR) image is available from this camera.
    */
    bool hasIRImage() const;
    /**
    * Returns true if a fisheye image is available from this camera.
    * @return true if a fisheye image is available from this camera.
    */
    bool hasFishEyeMap() const;

    /**
    * Gracefully closes the R200 camera.
    */
    void destroyInstance();
    /**
     * Get the FishEye Image from this camera, if available. Else, throws an error.
     * Type: CV_8UC1
     */
    cv::Mat getFishEyeMap() const;
    /**
     * Update the IMU measurment queue up until the given time
     */
    void updateImuToTime(double time);

    /**
     * Get the stored IMU measurments corresponding to the last camera frame
     * @return pointer to vector of ordered and syncronized imu measurements
     */
    std::vector<ImuPair> getImuData() const;

    const double getTimeStamp() const;





  private:

    cv::Point3f deproject(cv::Point2f pixelPoint, cv::Mat intrinsics, float depth);


    /**
    * Initializat the camera
    */
    void initCamera();
    void fillInZCoords();
    void fillInRGBImg();
    void fillInFishEyeImg();
    void initializeImages();
    static const int REAL_WID = 640, REAL_HI = 480;
    static const int IMU_SYNC_THRESHOLD =4;
    static const int FRAME_SYNC_THRESHOLD =15;
    rs::context mCtx;
    rs::device * mpDev;
    rs::intrinsics mDepth_intrin;
    rs::intrinsics mColor_intrin;
    cv::Mat depth_intrinsics;
    cv::Mat fishEyeMap;
    cv::Mat depthMap;
    std::vector<ImuPair> imuData;
    float mDepthScale;
    double mTimeStamp;
    double mTimeImu;
    SingleConsumerPriorityQueue<ImuMeasurement> imu_queue;
    SingleConsumerPriorityQueue<FrameObject> depth_queue;
    SingleConsumerPriorityQueue<FrameObject> fisheye_queue;
    SingleConsumerPriorityQueue<FrameObject> rgb_queue;


  };
}