/* modified OpenARK to generate test data for hand tracking
   (minimally changed to ensure data format is same as what OpenARK receives)*/

   // OpenARK Libraries
#include "core.h"
#include "SR300Camera.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace cv;

const std::string IMG_EXT = ".dm", TXT_EXT = ".txt",
CONTOUR_PATH = "contour/", DEPTH_PATH = "depth/";

static inline std::string makeOutputName(int id, int wid = 6) {
    std::stringstream sstm;
    sstm << setw(wid) << setfill('0') << id;
    return sstm.str();
}

template <class T>
static inline void writeBinary(std::ofstream & ofs, T val) {
    ofs.write(reinterpret_cast<const char *>(&val), sizeof(val));
}

int main(int argc, char** argv) {
    ark::DepthCamera * camera = new ark::SR300Camera();

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

    //auto handAverager = StreamingAverager(4, 0.1);
    //auto paleeteAverager = StreamingAverager(6, 0.05);

    printf("Welcome to the OpenARK Hand Identification Test Generator.\n(c) Alex Yu 2017\n");

    std::string oPathS = "";
    if (argc > 1) {
        oPathS = argv[1];
    }
    else {
        printf("Output path (leave empty to record into current directory):\n");
        std::getline(cin, oPathS);
    }

    if (oPathS == ".") oPathS = "";

    boost::filesystem::path oPath(oPathS);

    if (oPathS != "")
        boost::filesystem::create_directories(oPath);

    printf("\nPress space to start/stop recording. Press q on window to exit.\n");

    const std::string WIND_NAME = "OpenARK Object Identification Test Generator";
    namedWindow(WIND_NAME);

    bool recording = false;

    boost::filesystem::create_directories(oPath / CONTOUR_PATH);
    boost::filesystem::create_directories(oPath / DEPTH_PATH);

    camera->beginCapture(120);

    ark::ObjectParams params;
    params.handUseSVM = false;
    params.handRequireEdgeConnected = false;

    cv::Mat visual;

    // record data
    while (true)
    {
        if (camera->badInput()) {
            waitKey(10);
            continue;
        }

        std::string name;

        if (camera->hasIRMap()) {
            camera->getIRMap().convertTo(visual, CV_8UC1);
            visual /= 2;
            cv::cvtColor(visual, visual, cv::COLOR_GRAY2BGR, 3);
        }
        else {
            visual = cv::Mat::zeros(camera->getImageSize(), CV_8UC3);
        }

        // cluster & classifying objects in the scene
        const ark::Vec_Hand & hands = camera->getFrameHands(&params);

        for (uint i = 0; i < hands.size(); i++) {
            auto hand = hands[i];
            ark::Visualizer::visualizeHand(visual, visual, hand.get());

            if (recording) {
                name = makeOutputName(outputID++);

                std::ofstream ofsc((oPath / (CONTOUR_PATH + name + TXT_EXT)).string());
                std::vector<cv::Point> cont = hand->getContour();
                for (uint j = 0; j < cont.size(); ++j) {
                    ofsc << cont[j].x << " " << cont[j].y << "\n";
                }

                ofsc.close();
                cv::Mat dep = hand->getDepthMap();
                std::ofstream ofsd((oPath / (DEPTH_PATH + name + IMG_EXT)).string(), ios::binary | ios::out);

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
                                    writeBinary(ofsd, (float)(zrun + 1));
                                }
                                zrun = 0;

                                writeBinary(ofsd, ptr[j][0]); writeBinary(ofsd, ptr[j][1]); writeBinary(ofsd, ptr[j][2]);
                            }
                        }
                    }

                    ofsd.close();
                }

                if (outputID && outputID % 20 == 0) {
                    printf("Finished generating %d cases.\n", outputID);
                }
            }
        }

        cv::putText(visual, (recording ? "Recording" : "Paused (Press space to record)"), cv::Point(15, 25), 0, 0.5,
            cv::Scalar(255, 255, 255), 1, cv::LINE_8);
        cv::putText(visual, name, cv::Point(visual.cols - 80, 25), 0, 0.5,
            cv::Scalar(255, 255, 255), 1, cv::LINE_8);
        cv::imshow(WIND_NAME, visual);

        /**** End: Write Frames to File ****/

        /**** Start: Loop Break Condition ****/
        int c = cv::waitKey(1);
        if (c == ' ') recording = !recording;
        if (c == 'q' || c == 'Q' || c == 27) {
            break;
        }
        /**** End: Loop Break Condition ****/
    }

    cv::destroyAllWindows();
    return 0;
}