/* modified OpenARK to generate test data for hand tracking
   (minimally changed to ensure data format is same as what OpenARK receives)*/

#include "stdafx.h"

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

const std::string CONTOUR_PATH = "contour/", DEPTH_PATH = "depth/",
                  IMG_EXT = ".tsi", TXT_EXT = ".txt";

static inline std::string makeOutputName(int id, int wid = 6) {
    std::stringstream sstm;
    sstm << setw(wid) << setfill('0') << id;
    return sstm.str();
}

template <class T>
static inline void writeBinary(std::ofstream & ofs, T val) {
    ofs.write(reinterpret_cast<const char *>(&val), sizeof(val));
}

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

    boost::filesystem::create_directories(CONTOUR_PATH);
    boost::filesystem::create_directories(DEPTH_PATH);

    // record data
    while (true)
    {
        camera->nextFrame();

        if (camera->badInput) {
            waitKey(10);
            continue;
        }

        std::string name;

        cv::Mat visual;
        cv::resize(camera->getXYZMap(), visual, camera->getXYZMap().size() * 2);

        if (visual.cols == 0) {
            visual = cv::Mat::zeros(640, 480, CV_8UC3);
        }

        if (recording) {
            // cluster & classifying objects in the scene
            std::vector<Object3D> objects = camera->queryObjects();

            for (uint i = 0; i < objects.size(); i++) {
                Object3D obj = objects[i];
                if (!obj.hasHand) continue;

                name = makeOutputName(outputID++);
                
                std::ofstream ofsc(CONTOUR_PATH + name + TXT_EXT);

                std::vector<cv::Point> cont = obj.getContour();
                for (uint j = 0; j < cont.size(); ++j) {
                    ofsc << cont[j].x << " " << cont[j].y << "\n";
                }

                ofsc.close();

                cv::Mat dep = obj.getDepthMap();

                std::ofstream ofsd (DEPTH_PATH + name + IMG_EXT, ios::binary | ios::out);

                if (ofsd) {
                    writeBinary(ofsd, (ushort)dep.rows); writeBinary(ofsd, (ushort)dep.cols);

                    int zrun = 0;
                    for (int i = 0; i < dep.rows; ++i)
                    {
                        cv::Vec3f * ptr = dep.ptr<cv::Vec3f>(i);
                        for (int j = 0; j < dep.cols; ++j)
                        {
                            if (ptr[j][2] == 0) {
                                ++zrun;
                                continue;
                            }
                            else {
                                if (zrun >= 1) {
                                    writeBinary(ofsd, (float)(zrun+1));
                                }
                                zrun = 0;

                                writeBinary(ofsd, ptr[j][0]); writeBinary(ofsd, ptr[j][1]); writeBinary(ofsd, ptr[j][2]); }
                        }
                    }

                    ofsd.close();
                }

                cv::polylines(visual, obj.getContour(), true, cv::Scalar(0, 255, 0));

                cv::circle(visual, obj.getHand().centroid_ij, 15, cv::Scalar(255, 0, 0), 4);

                std::vector<cv::Point> & fingers = obj.getHand().fingers_ij;
                std::vector<cv::Point> & defects = obj.getHand().defects_ij;
                for (uint i = 0; i < fingers.size(); ++i) {
                    cv::line(visual, fingers[i], defects[i], cv::Scalar(0, 0, 255), 1);
                    cv::line(visual, obj.getHand().centroid_ij, defects[i], cv::Scalar(255, 0, 100), 1);

                    cv::circle(visual, fingers[i], 8, cv::Scalar(0, 0, 255), -1);
                    cv::circle(visual, defects[i], 6, cv::Scalar(255, 0, 100), -1);
                }
            }

            if (outputID && outputID % 20 == 0) {
                printf("Finished generating %d cases.\n", outputID);
            }
        }

        cv::resize(visual, visual, visual.size() / 2);

        if (recording) {
            cv::putText(visual, "Recording", cv::Point(15, 25), 0, 0.5, cv::Scalar(60, 255, 50));
        }
        else {
            cv::putText(visual, "Paused (Press space to record)", cv::Point(15, 25), 0, 0.5, cv::Scalar(50, 50, 255));
        }

        cv::putText(visual, name, cv::Point(visual.cols - 80, 25), 0, 0.5, cv::Scalar(255, 255, 255));

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