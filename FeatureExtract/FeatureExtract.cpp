#include<iostream>
#include<fstream>
#include<vector>
#include<climits>
#include<algorithm>
#include<string>
#define PI 3.1415926535898

#include "opencv2/opencv.hpp"
#include "boost/filesystem.hpp"

#include "FeatureExtract.h"
#include "ImageCleanRefit.h"
#include "Util.h"

using namespace boost::filesystem;

namespace classifier {
    namespace features {
        static inline float euclideanNorm(cv::Point2f a, cv::Point2f b) {
            return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
        }

        void computeMeanAndVariance(const cv::Mat& clean, cv::Vec3f center,
            double& avgdist, double& vardist, double& avgdepth, double& vardepth) {

            avgdist = avgdepth = 0;
            int totalpts = 0;

            for (int r = 0; r < clean.rows; ++r) {
                const cv::Vec3f * ptr = clean.ptr<cv::Vec3f>(r);
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3f pt = ptr[c];
                    if (pt[2] != 0) {
                        cv::Point2f xypt(pt[0], pt[1]);
                        avgdist += 
                            sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                        avgdepth += pt[2];
                        ++totalpts;
                    }
                }
            }

            if (totalpts == 0) {
                avgdist = avgdepth = 1.0;
                vardist = vardepth = 0.0;
                return;
            }

            avgdist /= totalpts;
            avgdepth /= totalpts;

            vardist = vardepth = 0;

            for (int r = 0; r < clean.rows; ++r) {
                const cv::Vec3f * ptr = clean.ptr<cv::Vec3f>(r);
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3f pt = ptr[c];
                    if (pt[2] != 0) {
                        cv::Point2f xypt(pt[0], pt[1]);
                        double dist = 
                            sqrtf((pt[0] - center[0]) * (pt[0] - center[0]) + (pt[1] - center[1]) * (pt[1] - center[1]));
                        vardist += (dist - avgdist) * (dist - avgdist);
                        vardepth += (pt[2] - avgdepth) * (pt[2] - avgdepth);
                    }
                }
            }

            vardist /= totalpts;
            vardepth /= totalpts;
        }

        static void circleSweep(const cv::Mat& clean, cv::Vec3f centerxyz,
            cv::Point centerij, double radius, double& coverage, double& comr, double& comd) {

            cv::Mat mask = cv::Mat::zeros(clean.rows, clean.cols, CV_8U);

            cv::Size2f sz((float)(clean.cols * radius), (float)(clean.rows * radius));
            cv::RotatedRect rect(centerij, sz, 0.0);

            cv::ellipse(mask, rect, cv::Scalar(255), 2);

            comr = 0; comd = 0;
            double avgx = 0, avgy = 0;
            int goodpts = 0, totalpts = 0;

            for (int r = 0; r < clean.rows; ++r) {
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3b pt = clean.at<cv::Vec3b>(r, c);
                    if (mask.at<uchar>(r, c)) {
                        cv::Point xypt(pt[0], pt[1]);
                        if (pt[2] != 0) {
                            avgx += pt[0];
                            avgy += pt[1];
                            ++goodpts;
                        }
                        ++totalpts;
                    }
                }
            }

            if (goodpts == 0) {
                comr = 0.0; comd = 0.0; coverage = 0.0;
                return;
            }

            avgx /= goodpts;
            avgy /= goodpts;

            cv::Point center = cv::Point(centerxyz[0], centerxyz[1]);

            cv::Point com(avgx, avgy);
            comr = sqrt(euclideanNorm(com, center));

            for (int r = 0; r < clean.rows; ++r) {
                for (int c = 0; c < clean.cols; ++c) {
                    cv::Vec3b pt = clean.at<cv::Vec3b>(r, c);
                    if (mask.at<uchar>(r, c) && pt[2] != 0) {
                        cv::Point xypt(pt[0], pt[1]);
                        comd += sqrt(euclideanNorm(xypt, com));
                    }
                }
            }

            comd /= goodpts;

            coverage = (double)goodpts / totalpts;
        }

        static inline cv::Point2f getCentroid(const cv::Mat & depthChannel) {
            auto m = cv::moments(depthChannel, false);
            if (m.m00 == 0) return cv::Point2f(depthChannel.cols / 2.0f, depthChannel.rows / 2.0f);
            return cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
        }

        static inline double contourDiameter(const std::vector<cv::Point> & cont) {
            double maxd;
            cv::Point startp, bestp;
            startp = bestp = cont[0];

            for (int _ = 0; _ < 2; ++_) {
                maxd = 0;
                for (auto p : cont) {
                    double curd = euclideanNorm(p, startp);
                    if (curd > maxd) {
                        bestp = p;
                        maxd = curd;
                    }
                }
                startp = bestp;
            }

            return sqrt(maxd);
        }

        static std::vector<int> histogram(const cv::Mat & chnl, int buckets = 16, int maxVal = 256) {
            int bucketSz = maxVal / buckets;

            std::vector<int> result(buckets);

            for (int r = 0; r < chnl.rows; ++r) {
                for (int c = 0; c < chnl.cols; ++c) {
                    uchar val = chnl.at<uchar>(r, c);
                    if (val) {
                        int bucket = val / bucketSz;
                        if (bucket >= buckets) bucket = buckets - 1;
                        ++result[bucket];
                    }
                }
            }

            return result;
        }

        int getNumFeatures(void) {
            return (int)REGULAR_FEATURE_NAMES.size() + (int)CIRCLE_SWEEP_RADII.size() * 3 +
                HISTOGRAM_XY_BUCKETS * 2 + HISTOGRAM_Z_BUCKETS;
        }

        std::vector<std::pair<double, double> > * loadScalingInfo(std::string scalePath) {
            auto result = new std::vector<std::pair<double, double> >;

            std::ifstream ifsscale;

            ifsscale = std::ifstream(scalePath);

            if (!ifsscale) {
                std::cerr << "WARNING: loadScalingInfo: Featue scaling file does not exist or is inaccessible." <<
                    " Tip: Specify 'scalingPath' manually when calling extractFeatures.";
                return result;
            }

            // ignore headings line
            std::string _; std::getline(ifsscale, _);

            for (int i = 0; i < getNumFeatures(); ++i) {
                double low, range;
                if (!(ifsscale >> low >> range)) {
                    std::cerr << "WARNING: loadScalingInfo: The feature scaling file " << scalePath <<
                        " describes less features than required!\n";
                    break;
                }

                result->push_back(std::make_pair(low, range));
            }

            ifsscale.close();

            return result;
        }


        std::vector<double> extractFeatures(const cv::Mat & depth,
            const std::vector<cv::Point> & contour,
            const std::vector<std::pair<double, double> > * scale,
            const cv::Mat & depthFit) {
            std::vector<double> feat;

            cv::Mat clean;
            if (depthFit.cols > 1) {
                clean = depthFit;
            }
            else {
                clean = data::cleanImage(depth, 4);
            }

            cv::Mat channels[3];
            cv::split(clean, channels);

            // compute some stuff
            std::vector<cv::Point> hull;
            convexHull(contour, hull);
            double conta = contourArea(contour);
            double contd = contourDiameter(contour);

            float cirr; cv::Point2f circ;
            minEnclosingCircle(contour, circ, cirr);
            double cira = PI * cirr * cirr;

            cv::Point2f center = getCentroid(channels[2]);
            cv::Vec3f centerXyz = Util::averageAroundPoint(depth, center, 15);

            double avgdist, vardist, avgdepth, vardepth;
            computeMeanAndVariance(clean, centerXyz, avgdist, vardist, avgdepth, vardepth);

            // BEGIN ADDING FEATURES TO FEATURE LIST

            // add contour perimeter
            feat.push_back(arcLength(contour, true));

            // add contour area
            feat.push_back(conta);

            // add contour extent    
            feat.push_back(conta / cira);

            // add contour diamter
            feat.push_back(contd);

            // add hull perimeter
            feat.push_back(cv::arcLength(hull, true));

            // add hull area
            feat.push_back(cv::contourArea(hull));

            // add number of pixels
            feat.push_back(cv::countNonZero(channels[2]));

            // add centroid distance from real center
            feat.push_back(sqrt(euclideanNorm(center, center)));

            // add average, stddev of xy distance & depth from centroid
            feat.push_back(avgdist);
            feat.push_back(sqrt(vardist));
            feat.push_back(avgdepth);
            feat.push_back(sqrt(vardepth));

            // add circle sweep features
            for (double d : CIRCLE_SWEEP_RADII) {
                double coverage, com_from_center, com_mean_dist;
                circleSweep(clean, centerXyz, center, d, coverage, com_from_center, com_mean_dist);

                feat.push_back(coverage);
                feat.push_back(com_from_center);
                feat.push_back(com_mean_dist);
            }

            // add histogram features
            std::vector<int> histx = histogram(channels[0], HISTOGRAM_XY_BUCKETS, 64),
                histy = histogram(channels[1], HISTOGRAM_XY_BUCKETS, 64),
                histz = histogram(channels[2], HISTOGRAM_Z_BUCKETS, 176);

            for (int i = 0; i < HISTOGRAM_XY_BUCKETS; ++i) {
                feat.push_back((double)histx[i]);
                feat.push_back((double)histy[i]);
            }

            for (int i = 0; i < HISTOGRAM_Z_BUCKETS; ++i) {
                feat.push_back((double)histz[i]);
            }

            for (int i = 0; i < feat.size(); ++i) {
                if (scale != nullptr) {
                    double low = (*scale)[i].first, range = (*scale)[i].second;

                    // eliminate NaN and replace with middle of range
                    if (isnan(feat[i])) feat[i] = low + range / 2.0;
                    else {
                        if (range == 0) feat[i] = 0;
                        else feat[i] = (feat[i] - low) / range;
                    }
                }
                else {
                    if (isnan(feat[i])) feat[i] = 0; // eliminate NaN and replace with zero
                }

            }

            return feat;
        }

        std::vector<double> extractFeatures(std::string testCaseName, std::string dataDir,
             std::string scalePath, std::string depthPath, std::string depthFitPath, std::string contourPath) {

            if (dataDir[dataDir.size() - 1] != '/' && dataDir[dataDir.size() - 1] != '\\') {
                dataDir += path::preferred_separator;
            }

            std::string dpp = dataDir + depthPath + testCaseName + IMG_EXT,
                dfp = dataDir + depthFitPath + testCaseName + IMG_EXT,
                ctp = dataDir + contourPath + testCaseName + TXT_EXT;

            std::ifstream ifscont(ctp);
            if (!ifscont) return std::vector<double>(); // invalid path

            std::vector<cv::Point> cont;

            // read contour info
            int x, y;
            while (ifscont >> x >> y) {
                cont.push_back(cv::Point(x, y));
            }

            ifscont.close();

            if (cont.size() <= 2) return std::vector<double>(); // invalid contour


            // read normal & cleaned depth maps
            cv::Mat depth = cv::imread(dpp), clean = cv::imread(dfp);


            std::vector<std::pair<double, double>> * scaleInfo = nullptr;
            if (scalePath != "") scaleInfo = loadScalingInfo(scalePath);

            auto result = extractFeatures(depth, cont, scaleInfo, clean);

            if (scaleInfo != nullptr) delete scaleInfo;

            return result;
        }

    }
}
