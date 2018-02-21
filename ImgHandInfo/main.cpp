#include <utility>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "core.h"
#include "HandFeatureExtractor.h"

using namespace boost::filesystem;

static std::string inPath, outPath, outFPath;
static std::ofstream ofsf;

const char CSV_SEP = '\t';

static void extractFeatures(std::string tcName) {
    std::vector<double> feats = classifier::features::extractHandFeatures(tcName, inPath);

    if (feats.size() > 0) {
        ofsf << tcName << CSV_SEP << feats.size();
        for (int i = 0; i < feats.size(); ++i) {
            if (isnan(feats[i])) ofsf << CSV_SEP << 1.0;
            else ofsf << CSV_SEP << feats[i];
        }
        ofsf << "\n";
    }
}

int main(int argc, char * argv[]) {
    if (argc >= 2) {
        inPath = argv[1];
    }
    else {
        std::cout << "Test case base directory:\n";
        std::cin >> inPath;
    }

    if (inPath[inPath.size() - 1] != '/' && inPath[inPath.size() - 1] != '\\') {
        inPath += path::preferred_separator;
    }

    outFPath = inPath + "handfeatures.csv";

    int count = 0;

    if (is_directory(inPath)) {
        directory_iterator end_itr;

        ofsf = std::ofstream(outFPath);
        ofsf << "id" << CSV_SEP << "num_features" << "\n";

        for (directory_iterator itr(inPath + "contour"); itr != end_itr; ++itr)
        {
            if (!is_directory(itr->status()))
            {
                std::string caseName = itr->path().stem().string();

                extractFeatures(caseName);

                if (++count % 20 == 0) {
                    std::cout << count << " images processed." << std::endl;
                }
            }
        }

        ofsf.close();
    }
    else {
        std::cerr << "ERROR: Path is not a directory!\n";
        return 1;
    }

    std::cout << "All done! " << count << " images processed." << std::endl;

    return 0;
}