#include <string>
#include <iostream>
#include <fstream>

#include "core.h"

template <class T>
static inline void readBinary(std::ifstream & ifs, T * val) {
    ifs.read(reinterpret_cast<char *>(val), sizeof(* val));
}

int main(int argc, char ** argv) {
    std::string path;
    if (argc > 1) {
        path = argv[1];
    }
    else {
        std::cout << "Image path:\n";
        std::getline(std::cin, path);
    }

    if (path.size() && path[0] == '"') {
        path = path.substr(1);
    }
    if (path.size() && path[path.size()-1] == '"') {
        path = path.substr(0, path.size()-1);
    }

    std::ifstream ifs (path, ios::binary | ios::in);

    if (!ifs) {
        std::cerr << "Couldn't open file.\n";
        return 1;
    }

    ushort wid, hi; 
    readBinary(ifs, &hi);
    readBinary(ifs, &wid);

    cv::Mat m = cv::Mat::zeros(hi, wid, CV_32FC3);

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

    cv::imshow(path, m);

    ark::ObjectParams params;
    params.handRequireEdgeConnected = false;
    params.handUseSVM = false;
    ark::Hand obj(m, &params);

    cv::Rect bounds = obj.getBoundingBox();

    std::cout << "3D OBJECT DATA\nSurface Area: " << obj.getSurfArea() << "\n\n";
    std::cout << "Bounding Box on Screen: " << bounds << "\n";
    std::cout << "Center on Screen: " << obj.getCenterIJ() << "\n";
    std::cout << "XYZ Center: " << obj.getCenter() << "\n\n";

    if (obj.getContour().size() > 2) {
        std::cout << "Points in Contour: " << obj.getContour().size() << "\n";
        std::cout << "Contour Area: " << cv::contourArea(obj.getContour(), false) << "\n";
        std::cout << "Contour Perimeter: " << cv::arcLength(obj.getContour(), true) << "\n";
    }

    if (obj.getConvexHull().size() > 2) {
        std::cout << "Points in Convex Hull: " << obj.getConvexHull().size() << "\n";
        std::cout << "Convex Hull Area: " << cv::contourArea(obj.getConvexHull(), false) << "\n";
        std::cout << "Convex Hull Perimeter: " << cv::arcLength(obj.getConvexHull(), true) << "\n";
    }

    int diagLen = ark::util::euclideanDistance(bounds.tl(), bounds.br()) + 1;
    cv::Mat cropped = m(bounds);

    cv::Mat normalMap, normalVis;
    ark::util::computeNormalMap(cropped, normalMap, 6, 3, true);

    cv::Vec3f avg(0, 0, 0);
    cv::Vec3f * ptr;

    // find average normal direction
    for (int i = 0; i < normalMap.rows; ++i) {
        ptr = normalMap.ptr<cv::Vec3f>(i);
        for (int j = 0; j < normalMap.cols; ++j) {
            avg += ptr[j];
        }
    }

    avg = cv::normalize(avg);
    std::cout << "\nAverage Normal Vector: " << avg << "\n";

    cv::Point2f direction = cv::Point2f(avg[0], -avg[1]);
    double angle = fmod(180.0 + ark::util::pointToAngle(direction) / PI * 180.0, 360.0);
    std::cout << "Correction Angle: " << angle << "\n";


    // find diameter
    const std::vector<cv::Point> & contour = obj.getContour();
    int diaA, diaB;
    ark::util::diameter(contour, diaA, diaB);

    if (contour[diaA].y < contour[diaB].y) {
        std::swap(diaA, diaB);
    }

    angle = 180.0 + ark::util::pointToAngle(contour[diaB] - contour[diaA]) / PI * 180.0;

    // rotate so that diameter is pointing up (on screen
    cv::Point center = cv::Point2f(bounds.width / 2, bounds.height / 2);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Rect bounds2 = cv::RotatedRect(center, bounds.size(), angle).boundingRect();
    bounds2.height = bounds2.width = std::max(bounds2.width, bounds2.height);

    cv::Point2f center2(bounds2.width / 2, bounds2.height / 2);

    rot.at<double>(0, 2) += center2.x - center.x; 
    rot.at<double>(1, 2) += center2.y - center.y;

    cv::Mat normalMap2;
    cv::warpAffine(normalMap, normalMap2, rot, bounds2.size());

    // crop off black area
    cv::Point2f tl(INT_MAX, INT_MAX), br(0,0);
    for (int i = 0; i < normalMap2.rows; ++i) {
        ptr = normalMap2.ptr<cv::Vec3f>(i);
        for (int j = 0; j < normalMap2.cols; ++j) {
            if (ptr[j] != cv::Vec3f(0, 0, 0)) {
                tl.x = std::min(tl.x, (float)j);
                tl.y = std::min(tl.y, (float)i);
                br.x = std::max(br.x, (float)j);
                br.y = std::max(br.y, (float)i);
            }
        }
    }

    float dwid = br.x - tl.x;
    float dhigh = br.y - tl.y;

    const float IM_FACT = 2.25;
    const int NUM_THETA_BINS = 9, NUM_PHI_BINS = 5, BLOCK_SIZE = 16,
              IM_WID = 128, IM_HIGH = IM_FACT * IM_WID;

    // fit to size
    if (dhigh < dwid * IM_FACT) dhigh = dwid * IM_FACT;
    else if (dwid < dhigh / IM_FACT) dwid = dhigh / IM_FACT;

    cv::Point2f mid = tl + (br - tl) / 2.0 ;
    tl = mid - cv::Point2f(dwid, dhigh) / 2.0;
    br = mid + cv::Point2f(dwid, dhigh) / 2.0;
    tl.x = std::max(tl.x, 0.0f); tl.y = std::max(tl.y, 0.0f);
    br.x = std::min(br.x, (float)normalMap2.cols-1);
    br.y = std::min(br.y, (float)normalMap2.rows-1);
    normalMap2 = normalMap2(cv::Rect(tl, br));

    cv::pyrUp(normalMap2, normalMap2);
    cv::resize(normalMap2, normalMap2, cv::Size(IM_WID, IM_HIGH));

    ark::Visualizer::visualizeNormalMap(normalMap2, normalVis, 1);

    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::vector<cv::KeyPoint> keyPts;
    cv::Mat gray;
    cv::cvtColor(normalVis, gray, cv::COLOR_BGR2GRAY);
    orb->detect(gray, keyPts);

    cv::resize(normalVis, normalVis, cv::Size(200, 200 * IM_FACT), 0, 0, cv::INTER_NEAREST);

    // extract features
    std::vector<int> features;
    for (int i = 0; i < IM_HIGH; i += BLOCK_SIZE) {
        for (int j = 0; j < IM_WID; j += BLOCK_SIZE) {
            cv::Mat bins = cv::Mat::zeros(NUM_THETA_BINS, NUM_PHI_BINS, CV_32S);

            cv::Vec3f * nptr;

            // construct histogram for each block
            for (int r = i; r < i + BLOCK_SIZE; ++r) {
                nptr = normalMap2.ptr<cv::Vec3f>(r);
                for (int c = j; c < j + BLOCK_SIZE; ++c) {
                    cv::Vec3f & norm = nptr[c];
                    if (norm == cv::Vec3f(0, 0, 0)) continue;

                    norm = cv::normalize(norm);
                    if (norm[2] > 0) norm = -norm;

                    cv::Point2f pt(norm[0], -norm[1]);
                    float theta = ark::util::pointToAngle(pt);
                    int thetaBin = theta / (PI * 2.0) * NUM_THETA_BINS;
                    thetaBin = std::min(std::max(thetaBin, 0), NUM_THETA_BINS - 1);

                    float magnitude = ark::util::magnitude(pt);
                    float phi = atan2(-norm[2], magnitude);

                    int phiBin = phi / PI * 2.0 * NUM_PHI_BINS;
                    phiBin = std::min(std::max(phiBin, 0), NUM_PHI_BINS - 1);

                    bins.at<int>(thetaBin, phiBin) += 1;
                }
            }

            int * ptr;
            for (int r = 0; r < NUM_THETA_BINS; ++r) {
                ptr = bins.ptr<int>(r);
                for (int c = 0; c < NUM_PHI_BINS; ++c) {
                    features.push_back(ptr[c]);
                }
            }
        }
    }  

    for (float i = 0; i < normalVis.rows; i += (float)BLOCK_SIZE / IM_HIGH * normalVis.rows) {
        if (i) cv::line(normalVis, cv::Point(0, i), cv::Point(normalVis.cols, i), cv::Scalar(100, 100, 255));
    }

    for (float i = 0; i < normalVis.cols; i += (float)BLOCK_SIZE / IM_WID * normalVis.cols) {
        if (i) cv::line(normalVis, cv::Point(i, 0), cv::Point(i, normalVis.rows), cv::Scalar(100, 100, 255));
    }

    cv::Mat origNormalVis;
    ark::Visualizer::visualizeNormalMap(normalMap, origNormalVis, 1);

    cv::imshow("Original Normal Map", origNormalVis);
    cv::imshow("Invariant Normal Map", normalVis);

    cv::drawKeypoints(gray, keyPts, gray, cv::Scalar(0, 255, 0), 0);
    std::cout << "ORB Keypoints on Normal: " << keyPts.size() << "\n";
    cv::resize(gray, gray, cv::Size(200, 200 * IM_FACT));
    cv::imshow("ORB Keypoints on Normal", gray);

    cv::Mat croppedVis, tmp[3];
    cv::split(cropped, tmp);
    tmp[2].convertTo(croppedVis, CV_8U, 255);
    orb->detect(croppedVis, keyPts);
    cv::drawKeypoints(croppedVis, keyPts, croppedVis, cv::Scalar(0, 255, 0), 0);
    std::cout << "ORB Keypoints on Depth Map: " << keyPts.size() << "\n";
    cv::imshow("ORB Keypoints on Depth map", croppedVis);

    if (obj.isValidHand()) {
        std::cout << "\nFound Hand with " << obj.getNumFingers() << " fingers. Opening visualizer...\n";

        cv::Mat visual; ark::Visualizer::visualizeXYZMap(m, visual);
        visual /= 2;

        std::cout << "Hand Centroid: " << obj.getPalmCenter() << "\n";

        const std::vector<cv::Point> & fingers = obj.getFingersIJ();
        const std::vector<cv::Point> & defects = obj.getDefectsIJ();
        for (uint i = 0; i < fingers.size(); ++i) {

            std::cout << "Finger: " << obj.getFingers()[i] <<
                " Defect:" << obj.getDefects()[i] <<
                " Length:" <<
                ark::util::euclideanDistance(obj.getDefects()[i],
                    obj.getFingers()[i]) << "\n";
        }

        cv::line(visual, obj.getPalmCenterIJ(), ark::Point2f(obj.getPalmCenterIJ()) + obj.getDominantDirection() * 100,
            cv::Scalar(0, 255,255), 2);

        ark::Visualizer::visualizeHand(visual, visual, &obj, obj.getSurfArea());

        cv::imshow("Hand Detection", visual);
        std::cout << "Press any key to exit.\n";
    }

    else {
        std::cout << "No hand found. Press any key to exit.\n";
    }

    //std::cout << "FEATURE VECTOR:";
    //const int PER_CELL = NUM_THETA_BINS * NUM_PHI_BINS;
    //const int PER_ROW = IM_WID / BLOCK_SIZE * PER_CELL;

    //for (int i = 0; i < IM_HIGH / BLOCK_SIZE * PER_ROW; i += PER_ROW) {
    //    std::cout << "\n";
    //    for (int off = 0; off < PER_CELL; off += NUM_PHI_BINS) {
    //        std::cout << "\n";
    //        for (int j = 0; j < PER_ROW; j += PER_CELL) {
    //            int jj = i + j + off;
    //            for (int k = 0; k < NUM_PHI_BINS; ++k) {
    //                if (features[jj + k] < 10) {
    //                    std::cout << " ";
    //                }
    //                if (features[jj + k] < 100) {
    //                    std::cout << " ";
    //                }
    //                std::cout << features[jj + k] << " ";
    //            }
    //            std::cout << " ";
    //        }
    //    }
    //}
    //std::cout << "\n\n";

    cv::waitKey();

    return 0;
}