#include<iostream>
#include<boost/filesystem.hpp>

#include "FeatureExtract.h"
using namespace boost::filesystem;

const char PSEP = path::preferred_separator;

const std::string DEPTH_DIR = std::string("depth") + PSEP, DEPTH_FIT_DIR = std::string("depth_fit") + PSEP,
    CONTOUR_DIR = std::string("contour") + PSEP, FEAT_FILE = "features.csv",
    FEAT_SCALE_FILE = "scale.csv",
    FIRSTCOL_NAME = "id";

const char CSVSEP = ' ';

static std::vector <std::string> caseNames;
static std::vector <std::vector<double>> features;

static std::string inPath, depthPath, depthFitPath, contourPath, outPath, scaleOutPath;

const std::vector <double> CIRCLE_SWEEP_RADII = { 0.25, 0.5, 0.75 };
const int HISTOGRAM_BUCKETS = 8;

int main(int argc, char * argv[]) {
    std::cout << "Welcome to the feature extractor (c) Alex Yu 2017.\nThis program extracts features from images.\n";

    if (argc <= 1) {
        std::cout << "Path to root directory of test (should contain directory depth, depth_fit, etc.):\n";
        std::getline(std::cin, inPath);
    }
    else {
        inPath = argv[1];
    }

    if (!exists(inPath) || !is_directory(inPath)) {
        std::cerr << "ERROR: The specified path does not exist or is not a directory.\n";
        return 0;
    }

    if (inPath[inPath.size() - 1] != '/' && inPath[inPath.size() - 1] != '\\') {
        inPath += PSEP;
    }

    depthPath = inPath + DEPTH_DIR;
    depthFitPath = inPath + DEPTH_FIT_DIR;
    contourPath = inPath + CONTOUR_DIR;
    outPath = inPath + FEAT_FILE;
    scaleOutPath = inPath + FEAT_SCALE_FILE;

    if (!exists(depthPath) || !is_directory(depthPath) || !exists(depthFitPath) ||
        !is_directory(depthFitPath) || !exists(contourPath) || !is_directory(contourPath)) {
        std::cerr <<
            "ERROR: The directory specified is invalid because it does not contain one of 'depth' 'depth_fit' or 'contour'.\n";
        return 0;
    }

    int total = 0;

    features.resize(classifier::features::getNumFeatures());

    std::ofstream ofsOut(outPath), ofsScale(scaleOutPath);

    // scaling file headers
    ofsScale << "lower_bound" << CSVSEP << "range" << "\n";

    // output headers
    ofsOut << FIRSTCOL_NAME;

    for (unsigned i = 0; i < classifier::features::REGULAR_FEATURE_NAMES.size(); ++i) {
        ofsOut << CSVSEP << classifier::features::REGULAR_FEATURE_NAMES[i];
    }

    for (unsigned i = 0; i < classifier::features::CIRCLE_SWEEP_RADII.size(); ++i) {
        int radius = (int)(classifier::features::CIRCLE_SWEEP_RADII[i] * 100);
        ofsOut << CSVSEP << "cover_" << radius;
        ofsOut << CSVSEP << "comr_" << radius;
        ofsOut << CSVSEP << "comd_" << radius;
    }

    for (unsigned i = 0; i < classifier::features::HISTOGRAM_XY_BUCKETS; ++i) {
        ofsOut << CSVSEP << "histo_x_" << i;
        ofsOut << CSVSEP << "histo_y_" << i;
    }

    for (unsigned i = 0; i < classifier::features::HISTOGRAM_Z_BUCKETS; ++i) {
        ofsOut << CSVSEP << "histo_z_" << i;
    }

    ofsOut << std::endl;

    directory_iterator end_itr;

    std::cout << "Extracting features from images...\n";

    for (directory_iterator itr(contourPath); itr != end_itr; ++itr)
    {
        if (!is_directory(itr->status()))
        {
            std::string cur = itr->path().stem().string();
            caseNames.push_back(cur);

            auto feats = classifier::features::extractFeatures(cur, inPath);

            if (feats.size() != classifier::features::getNumFeatures()) {
                std::cerr << "WARNING: length of feature list for case " << cur <<
                    " does not match total number of features (" << feats.size()
                    << " features in case vs. " << classifier::features::getNumFeatures() << " features) \n";
                continue;
            }

            for (int i = 0; i < feats.size(); ++i) {
                features[i].push_back(feats[i]);
            }

            ++total;
            if (total % 20 == 0) {
                std::cout << "Extracted features from " << total << " images." << std::endl;
            }
        }
    }

    if (caseNames.size() == 0) {
        std::cout << "No images found. Exiting..." << std::endl;
        return 0;
    }

    std::cout << "\nRe-scaling features as necessary..." << std::endl;

    for (int i = 0; i < classifier::features::getNumFeatures(); ++i) {
        double minv = features[i][0], maxv = features[i][0];

        for (unsigned j = 1; j < features[i].size(); ++j) {
            minv = std::min(minv, features[i][j]);
            maxv = std::max(maxv, features[i][j]);
        }

        double maxabsv = std::max(abs(minv), abs(maxv));

        // scale this feature, if values are very large or very small, 
        // or the difference between max/min is very small
        if (maxabsv < 0.05 || maxabsv > 20 || maxv - minv < maxv * 0.1) {
            for (unsigned j = 0; j < features[i].size(); ++j) {
                if (maxv == minv) {
                    features[i][j] = 0;
                }
                else {
                    features[i][j] = (features[i][j] - minv) / (maxv - minv);
                }
            }

            ofsScale << minv << CSVSEP << maxv - minv << "\n";
        }
        else {
            ofsScale << 0 << CSVSEP << 1 << "\n";
        }
    }

    std::cout << "\nWritting results to features.txt..." << std::endl;

    for (unsigned i = 0; i < caseNames.size(); ++i) {
        ofsOut << caseNames[i];

        for (int j = 0; j < classifier::features::getNumFeatures(); ++j) {
            ofsOut << CSVSEP << features[j][i];
        }

        ofsOut << "\n";
    }

    ofsOut.close();
    ofsScale.close();

    std::cout << "All done. Extracted  " << classifier::features::getNumFeatures() <<
        " features from each of " << total << " images.\n";
}