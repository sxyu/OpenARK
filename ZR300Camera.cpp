#include "stdafx.h"
#include "version.h"
#include "ZR300Camera.h"
#include "Visualizer.h"

namespace ark {
    ZR300Camera::ZR300Camera(bool require_motion) :
        mTimeStamp(0), mTimeImu(0), requireMotion(require_motion) {
        rs::log_to_console(rs::log_severity::warn);
        initCamera();
    }

    void ZR300Camera::initCamera() {
        if (mCtx.get_device_count() == 0) exit(EXIT_FAILURE);

        // ensure a device is connected
        if (mCtx.get_device_count() == 0) {
            printf("No device connected\n");
            exit(EXIT_FAILURE);
        }

        // connect to a device
        // currently only supporting a single connected device
        mpDev = mCtx.get_device(0);
        printf("\nUsing device 0, an %s\n", mpDev->get_name());
        printf("    Serial number: %s\n", mpDev->get_serial());
        printf("    Firmware version: %s\n", mpDev->get_firmware_version());
        // ensure motion tracking is available
        if (!mpDev->supports(rs::capabilities::motion_events))
        {
            printf("This device does not support motion tracking!");
            exit(EXIT_FAILURE);
        }

        // setup callbacks for motion module
        auto motion_callback = [this](rs::motion_data entry)
        {
            //Queue the data, do not want to process in callback
            ImuMeasurement m;
            m.data = Eigen::Vector3d(entry.axes[0], entry.axes[1], entry.axes[2]);
            m.timestamp = entry.timestamp_data.timestamp;
            if (entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO) {
                m.type = ImuMeasurementType::gyro;
            }
            else {
                m.type = ImuMeasurementType::accel;
            }
            this->imu_queue.enqueue(m);
        };

        auto timestamp_callback = [](rs::timestamp_data entry)
        {
            // I'm not really sure why this is a thing but it is necessary for motion tracking
        };


        auto fisheye_callback = [this](rs::frame frame) {
            //Queue the data, do not want to process in callback
            FrameObject data;
            data.frame = std::make_shared<rs::frame>(std::move(frame));
            this->fisheye_queue.enqueue(data);
        };

        auto rgb_callback = [this](rs::frame frame) {
            //Queue the data, do not want to process in callback
            FrameObject data;
            data.frame = std::make_shared<rs::frame>(std::move(frame));
            this->rgb_queue.enqueue(data);
        };

        auto depth_callback = [this](rs::frame frame){
            //Queue the data, do not want to process in callback
            FrameObject data;
            data.frame = std::make_shared<rs::frame>(std::move(frame));
            this->depth_queue.enqueue(data);
        };


        std::cout << "enabling streams" << std::endl << std::flush; 
        // Configure depth and color to run with the device's preferred settings
        mpDev->enable_stream(rs::stream::color, rs::preset::best_quality);
        mpDev->enable_stream(rs::stream::depth, rs::preset::largest_image);

        // Enable fisheye camera
        mpDev->enable_stream(rs::stream::fisheye, REAL_WID, REAL_HI, rs::format::raw8, 30);

        mpDev->enable_motion_tracking(motion_callback, timestamp_callback);

        // Add callbacks to frame
        mpDev->set_frame_callback(rs::stream::fisheye, fisheye_callback);
        mpDev->set_frame_callback(rs::stream::color, rgb_callback);
        mpDev->set_frame_callback(rs::stream::depth, depth_callback);

        // Enable fisheye time alignment to motion events
        mpDev->set_option(rs::option::fisheye_strobe, 1);
        // Syncronize depth and fisheye frames
        mpDev->set_option(rs::option::fisheye_external_trigger, 1);
        // Enable fisheye auto exposure
        mpDev->set_option(rs::option::fisheye_color_auto_exposure , 1);

        std::cout << "starting device" << std::endl << std::flush; 
        mpDev->start(rs::source::all_sources);

        //std::cout << "waiting for first frames" << std::endl << std::flush; 
        //mpDev->wait_for_frames();

        mDepth_intrin = mpDev->get_stream_intrinsics(rs::stream::depth);
        mColor_intrin = mpDev->get_stream_intrinsics(rs::stream::color);
        
        /*/ <- add/remove * after the first slash to toggle calibration code.
        // This prints camera/IMU intrinsics you can put in intr.y[a]ml
        rs::intrinsics fish_intr = mpDev->get_stream_intrinsics(rs::stream::fisheye);
        Eigen::Matrix4f tf_imu_fish;
        rs::motion_intrinsics imu_intr = mpDev->get_motion_intrinsics();
        rs::extrinsics extr_imu_fisheye = mpDev->get_motion_extrinsics_from(rs::stream::fisheye);
        std::cerr << std::fixed << std::setprecision(10);
        std::cerr << "CAMERA\n";
        std::cerr << "distortion_coefficients\n";
        std::cerr << fish_intr.coeffs[0] << " " << fish_intr.coeffs[1] << " " << fish_intr.coeffs[2] << " " << fish_intr.coeffs[3] << "\n";
        
        std::cerr << "focal\n";
        std::cerr << fish_intr.fx << " " << fish_intr.fy << "\n";

        std::cerr << "principal\n";
        std::cerr << fish_intr.ppx << " " << fish_intr.ppy << "\n";

        tf_imu_fish << extr_imu_fisheye.rotation[0], extr_imu_fisheye.rotation[1], extr_imu_fisheye.rotation[2], extr_imu_fisheye.translation[0],
                       extr_imu_fisheye.rotation[3], extr_imu_fisheye.rotation[4], extr_imu_fisheye.rotation[5], extr_imu_fisheye.translation[1],
                       extr_imu_fisheye.rotation[6], extr_imu_fisheye.rotation[7], extr_imu_fisheye.rotation[8], extr_imu_fisheye.translation[2],
                       0.0, 0.0, 0.0, 1.0;
        std::cerr << "T_CS (Please invert for T_SC)\n";
        std::cerr << tf_imu_fish
        std::cerr << "\nMOTION\n";

        std::cerr << "a_max " << 176.0
                  << "\ng_max " << 7.8
                  << "\nsigma_g_c " << sqrt(imu_intr.gyro.noise_variances[0])
                  << "\nsigma_a_c " << sqrt(imu_intr.acc.noise_variances[0])
                  << "\nsigma_bg " << sqrt(imu_intr.gyro.bias_variances[0])
                  << "\nsigma_ba " << sqrt(imu_intr.acc.bias_variances[0])
                  << "\nsigma_gw_c " << 4.0e-6
                  << "\nsigma_aw_c " << 4.0e-5
                  << "\ntau " << 3600.0
                  << "\ng " << 9.81007
                  << "\na0 " << "[0, 0, 0]"
                  << "\nimu_rate" << 200;
        //*/ // END of calibration code, do not remove

        mDepthScale = mpDev->get_depth_scale();
        depth_intrinsics = cv::Mat::zeros(3, 3, CV_32FC1);
        depth_intrinsics.at<float>(0, 0) = mColor_intrin.fx;
        depth_intrinsics.at<float>(0, 1) = 0.0f; //Wrong, need change to s
        depth_intrinsics.at<float>(1, 1) = mColor_intrin.fy;
        depth_intrinsics.at<float>(0, 2) = mColor_intrin.ppx;
        depth_intrinsics.at<float>(1, 2) = mColor_intrin.ppy;
        depth_intrinsics.at<float>(2, 2) = 1.0f;

        // get rid of IMU data that queued while waiting for first frame
        // mTimeStamp = mpDev->get_frame_timestamp(rs::stream::fisheye);
        // std::cout << "going through imu queue" << std::endl << std::flush; 
        // updateImuToTime(mTimeStamp);
        fisheye_queue.clear();
        rgb_queue.clear();
        depth_queue.clear();
        imu_queue.clear();

    }

    ZR300Camera::~ZR300Camera() {
        // Stop the device
        if (mpDev->is_streaming())
            mpDev->stop(rs::source::all_sources);

        // Stop the motion tracker
        mpDev->disable_motion_tracking();
    }

    void ZR300Camera::update(MultiCameraFrame & frame) {
        FrameObject fish, rgb, depth;
        bool sync_found = false;

        while(!sync_found){
            while(!fisheye_queue.try_dequeue(&fish)){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            while(!rgb_queue.try_dequeue(&rgb)){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            while(!depth_queue.try_dequeue(&depth)){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (fabs(fish - rgb) < FRAME_SYNC_THRESHOLD && fabs(fish - depth) < FRAME_SYNC_THRESHOLD) {
                sync_found=true;
            } else {
                //if fisheye queue is ahead of rgb need to wait for new rgb
                if (fish - rgb > FRAME_SYNC_THRESHOLD) {
                    fisheye_queue.enqueue(fish);
                    //check to see if we also need to wait for depth
                    if (fish - depth < FRAME_SYNC_THRESHOLD)
                        depth_queue.enqueue(depth);
                } else { //fisheye queue is not ahead of rgb
                    //if rgb is ahead of fisheye need to wait for new fisheye
                    if (rgb - fish > FRAME_SYNC_THRESHOLD) {
                        rgb_queue.enqueue(rgb);
                        //check if we also need to wait for depth
                        if (rgb - depth < FRAME_SYNC_THRESHOLD)
                            depth_queue.enqueue(depth);
                    } else { //rgb is not ahead of fisheye
                        //if fisheye is ahead of depth we need to wait for new depth
                        if (fish - depth > FRAME_SYNC_THRESHOLD) {
                            fisheye_queue.enqueue(fish);
                            //we already know fisheye is not ahead of rgb queue so don't need to wait 
                            rgb_queue.enqueue(rgb);
                        } else if (depth - fish > FRAME_SYNC_THRESHOLD) {
                            //if depth is ahead of fisheye we need to wait for new fisheye
                            depth_queue.enqueue(depth);
                        }
                        //we already know rgb is NOT ahead of fisheye so we do need to wait for that
                    }
                }
            }
        }


        // get pointers to frames
        uint16_t * depth_image = (uint16_t *) depth.frame->get_data();
        uint8_t * rgb_image = (uint8_t *) rgb.frame->get_data();
        uint8_t * fish_image = (uint8_t *) fish.frame->get_data();

        mTimeStamp = fish.frame->get_timestamp();

        frame.images.resize(3);
        if (frame.images[0].empty()) frame.images[0] = cv::Mat(REAL_HI, REAL_WID, CV_32FC3);
        if (frame.images[1].empty()) frame.images[1] = cv::Mat(REAL_HI, REAL_WID, CV_8UC3);
        if (frame.images[2].empty()) frame.images[2] = cv::Mat(REAL_HI, REAL_WID, CV_8UC1);

        // populate RGB image
        std::memcpy((void*) frame.images[1].datastart, rgb_image, REAL_HI * REAL_WID * 3 * sizeof(unsigned char));
        // populate FishEye image
        std::memcpy((void*) frame.images[2].datastart, fish_image, REAL_HI * REAL_WID * sizeof(unsigned char));

        // Populate xyzMap
        for (int dy = 0; dy < REAL_HI; ++dy) {
            cv::Vec3f * ptr = frame.images[0].ptr<cv::Vec3f>(dy);
            for (int dx = 0; dx < REAL_WID; ++dx) {
                uint16_t depth_value = depth_image[dy * REAL_WID + dx];
                ptr[dx] = deproject(cv::Point2f(dx, dy), depth_intrinsics, depth_value * mDepthScale);
            }
        }

        updateImuToTime(mTimeStamp);
    }

    bool ZR300Camera::hasRGBMap() const {
        return true;
    }

    const cv::Mat ZR300Camera::getRGBMap() const
    {
        return getFrameImage(1, CV_8UC3);
        return cv::Mat();
    }

    bool ZR300Camera::hasFishEyeMap() const {
        return true;
    }

    const cv::Mat ZR300Camera::getFishEyeMap() const
    {
        return getFrameImage(2, CV_8UC1);
    }

    void ZR300Camera::updateImuToTime(double time) {
        imuData.clear();
        Eigen::Vector3d gyr(0,0,0);
        Eigen::Vector3d acc(0,0,0);
        double g_time(0);
        double a_time(0);
        ImuMeasurement imu_value;
        while(mTimeImu <=time){
            while(imu_queue.try_dequeue(&imu_value)){
                if(imu_value.type == ImuMeasurementType::gyro){
                    gyr = imu_value.data;
                    g_time = imu_value.timestamp;
                }
                else{
                    acc = imu_value.data;
                    a_time = imu_value.timestamp;
                }
                if(g_time!=0 && a_time!=0 && fabs(g_time-a_time)<IMU_SYNC_THRESHOLD){
                    mTimeImu = (g_time+a_time)/2;
                    imuData.push_back({mTimeImu,gyr,acc});
                    g_time = a_time = 0;
                }
            }
        }
    }

    std::vector<ImuPair> ZR300Camera::getImuData() const
    {
        return imuData;
    }

    const double ZR300Camera::getTimeStamp() const
    {
        return mTimeStamp;
    }

    cv::Point3f ZR300Camera::deproject(cv::Point2f pixelPoint, cv::Mat intrinsics, float depth) {
        float x = (pixelPoint.x - intrinsics.at<float>(0, 2)) / intrinsics.at<float>(0, 0);
        float y = (pixelPoint.y - intrinsics.at<float>(1, 2)) / intrinsics.at<float>(1, 1);
        // Didn't take into account of distortions
        cv::Point3f ThreeDLocation;
        ThreeDLocation.x = x * depth;
        ThreeDLocation.y = y * depth;
        ThreeDLocation.z = depth;
        return ThreeDLocation;
    }
} //namespace ark
