/* modified OpenARK to generate test data for hand tracking
   (minimally changed to ensure data format is same as what OpenARK receives)*/

// C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>

// OpenCV Libraries
#include "opencv2/highgui/highgui.hpp"

// OpenARK Libraries
#include "version.h"
#ifdef PMDSDK_ENABLED
    #include "PMDCamera.h"
#endif
#ifdef RSSDK_ENABLED
    #include "SR300Camera.h"
#endif
#include "Webcam.h"
#include "Visualizer.h"
#include "Hand.h"
#include "Plane.h"
#include "Calibration.h"
#include "Util.h"
#include "Object3D.h"
#include "StreamingAverager.h"

using namespace cv;

int main() {
    DepthCamera * camera = nullptr;

#ifdef PMDSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "pmd")) {
        camera = new PMDCamera();
    }
    else {
        return 0;
    }
#endif
#ifdef RSSDK_ENABLED
    if (!strcmp(OPENARK_CAMERA_TYPE, "sr300")) {
        camera = new SR300Camera();
    }
    else {
        return 0;
    }
#endif

    // unique ID for test case
    int outputID = 0;

    //RGBCamera *cam = new Webcam(1);
    //Calibration::XYZToUnity(*pmd, 4, 4, 3);
    FileStorage fs;
    fs.open("RT_Transform.txt", FileStorage::READ);

    Mat r, t;
    fs["R"] >> r;
    fs["T"] >> t;

    fs.release();

    //namedWindow("Results", CV_WINDOW_NORMAL);

    //auto handAverager = StreamingAverager(4, 0.1);
    //auto paleeteAverager = StreamingAverager(6, 0.05);

    printf("Welcome to the OpenARK Object Identification Test Generator.\n(c) Alex Yu 2017\nPlease enter starting test number: ");

    scanf("%d", &outputID);

    int startingID = outputID;

    printf("\nPress space to start/stop recording. Press q on window to exit.\n");

    namedWindow("OpenARK Object Identification Test Generator");
    
    bool recording = false;

    // record data
    while (true)
    {
        camera->update();

        // Loading image from sensor
        camera->removeNoise();

        if (camera->badInput) {
            waitKey(10);
            continue;
        }

        if (recording) {
            // cluster & classifying objects in the scene
            camera->computeClusters(0.667, 750, 15);
            auto clusters = camera->getClusters();

            for (auto i = 0; i < clusters.size(); i++) {
                Object3D(clusters[i].clone(), outputID++);
            }


            if (outputID && outputID % 20 == 0) {
                printf("Finished generating %d cases.\n", outputID);
            }
        }

        cv::Mat visual = camera->getXYZMap().clone();

        if (visual.cols == 0) {
            visual = cv::Mat::zeros(640, 480, CV_8UC3);
        }

        if (recording) {
            cv::putText(visual, "Recording", cv::Point(15, 25), 0, 0.5, cv::Scalar(60, 255, 50));
        }
        else {
            cv::putText(visual, "Paused (Press space to record)", cv::Point(15, 25), 0, 0.5, cv::Scalar(50, 50, 255));
        }

        cv::putText(visual, std::to_string(outputID), cv::Point(visual.cols- 30, 25), 0, 0.5, cv::Scalar(255, 255, 255));

        imshow("OpenARK Object Identification Test Generator", visual);

        /**** End: Write Frames to File ****/

        /**** Start: Loop Break Condition ****/
        auto c = waitKey(1);
        if (c == ' ') recording = !recording;
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }
        /**** End: Loop Break Condition ****/
    }

    camera->destroyInstance();
    destroyAllWindows();
    return 0;
}