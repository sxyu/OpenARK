#include "HandClassifier.h"
#include "core.h"

using namespace ark::classifier;

const char * trainingPath = "D:\\Programming\\3VR\\ht_tests_v3\\train\\";
const std::string benchmarkPaths[] = { "D:\\Programming\\3VR\\ht_tests_v3\\test\\",  "D:\\Programming\\3VR\\ht_tests_v3\\train\\", 
"D:\\Programming\\3VR\\ht_tests_v2\\test\\", "D:\\Programming\\3VR\\ht_tests_v2\\train\\"};
const char * savePath = "D:\\Programming\\3VR\\OpenARK\\svm\\";

int main(void) {
    int ihyperparams[5 * SVMHandClassifier::NUM_SVMS];

    std::string names[] = { "gamma", "coef0", "C", "eps", "p" };

    for (int i = 0; i < SVMHandClassifier::NUM_SVMS; ++i) {
        cv::namedWindow(std::string("SVM ") + (char)('0' + i));
    }

    for (unsigned i = 0; i < 5 * SVMHandClassifier::NUM_SVMS; ++i) {
        std::string windowName = "SVM "; windowName += (char)('0' + i / 5);

        if (i % 5 == 3) {
            ihyperparams[i] = (int)(SVMHandClassifier::DEFAULT_HYPERPARAMS[i] * 1e17);
            cv::createTrackbar(names[i % 5], windowName, &ihyperparams[i], ihyperparams[i] * 4);
        }
        else {
            ihyperparams[i] = (int)(SVMHandClassifier::DEFAULT_HYPERPARAMS[i] * 10000.0);
            if (i % 5 == 0) 
                cv::createTrackbar(names[i % 5], windowName, &ihyperparams[i], 100000.0);
            else
                cv::createTrackbar(names[i % 5], windowName, &ihyperparams[i], 10000.0);
        }

    }

    int boundary = 500;
    cv::createTrackbar("Boundary", "SVM 0", &boundary, 1000);

    double hyperparams[SVMHandClassifier::NUM_SVMS * 5];

    boost::shared_ptr<HandClassifier> hc = boost::make_shared<SVMHandClassifier>(savePath);

    while (true) {
        std::cout << "OpenARK SVM training tool: Press q / ESC to quit,\nl to load last saved configuration, 0-9 to run benchmark, or t to try training again...\n";

        char c = cv::waitKey();
        if (c == 'q' || c == 27) {
            break;
        }
        else if (c == 'l') {
            hc->loadFile(savePath);
        }
        else if (c >= '0' && c <= '9') {
            std::cout << "Running benchmark...\n";

            int benchIdx = c - '1';
            if (c < 0) c = 10;

            std::ifstream ifs(benchmarkPaths[benchIdx] + "labels.txt");
            std::ifstream ifsFeat(benchmarkPaths[benchIdx] + "handfeatures.csv");

            std::string _; std::getline(ifsFeat, _);

            int n; ifs >> n;
            double totalCorrect = 0, zeros = 0;

            double svmCorrect[SVMHandClassifier::NUM_SVMS],
                svmZeros[SVMHandClassifier::NUM_SVMS], svmTotal[SVMHandClassifier::NUM_SVMS],
                svmFalsePos[SVMHandClassifier::NUM_SVMS], svmFalseNeg[SVMHandClassifier::NUM_SVMS];

            memset(svmCorrect, 0, sizeof svmCorrect);
            memset(svmZeros, 0, sizeof svmZeros);
            memset(svmTotal, 0, sizeof svmTotal);
            memset(svmFalsePos, 0, sizeof svmFalsePos);
            memset(svmFalseNeg, 0, sizeof svmFalseNeg);

            int numFalseNeg = 0, numFalsePos = 0, total = 0;

            for (int i = 0; i < n; ++i) {
                std::vector<float> feats;
                std::string name; int nFeat, nFingers;

                if (!(ifsFeat >> name >> nFeat >> nFingers)) break;

                feats.push_back(nFingers);
                for (int i = 1; i < nFeat; ++i) {
                    double ft; ifsFeat >> ft;
                    feats.push_back(ft);
                }

                std::string lab = ""; int correct;
                while (lab != name && (ifs >> lab >> correct)) {};

                if (feats.size() == 0) continue;

                cv::Mat mFeats (1, feats.size(), CV_32F);
                for (uint i = 0; i < feats.size(); ++i) {
                    mFeats.at<float>(i) = feats[i];
                }

                int svmIdx = ark::classifier::SVMHandClassifier::getSVMIdx(mFeats);
                if (svmIdx < 0) continue;

                ++total;
                double result = hc->classify(mFeats);

                ++svmTotal[svmIdx];

                if (result < (boundary / 1000.0)) {
                    ++zeros;
                    ++svmZeros[svmIdx];
                }

                double confidence = std::min(abs(result - boundary / 1000.0) / abs(correct - boundary / 1000.0), 1.0);

                int prediction = (result >= boundary / 1000.0 ? 1 : 0);

                if (prediction == correct) {
                    ++totalCorrect;
                    ++svmCorrect[svmIdx];
                }
                else {
                    std::cout << "#" << lab << " (processed by SVM " << svmIdx << "): predicted " << prediction << " with confidence " <<
                        confidence << " but correct label is " << correct << "\n";

                    if (prediction == 1) {
                        ++numFalsePos;
                        ++svmFalsePos[svmIdx];
                    }
                    else {
                        ++numFalseNeg;
                        ++svmFalseNeg[svmIdx];
                    }
                }
            }

            std::cout << std::endl;
            for (int i = 0; i < SVMHandClassifier::NUM_SVMS; ++i) {
                std::cout << "SVM " << i << ":\n";
                std::cout << "\t" << svmTotal[i] << " Total Cases.\n";
                std::cout << "\t" << svmCorrect[i] / svmTotal[i] * 100.0 << "% Correctly Predicted.\n";
                std::cout << "\t" << svmZeros[i] / svmTotal[i] * 100.0 << "% Zeros.\n";
                std::cout << "\t" << svmFalsePos[i] << " False Positives, " << svmFalseNeg[i] << " False Negatives.\n";
            }

            std::cout << "\nOverall:\n";
            std::cout << totalCorrect / total * 100.0 << "% Correctly Predicted.\n";
            std::cout << zeros / total * 100.0 << "% of predictions are zeros\n\n";
            std::cout << numFalsePos << " False Positives, " << numFalseNeg << " False Negatives." << std::endl;

            ifs.close();
            ifsFeat.close();
        }
        else if (c == 't') {
            for (unsigned i = 0; i < 5 * SVMHandClassifier::NUM_SVMS; ++i) {
                if (i % 5 == 3)
                    hyperparams[i] = (double)ihyperparams[i] / 1e17;
                else
                    hyperparams[i] = (double)ihyperparams[i] / 10000;
            }

            std::cout << "Training with hyperparameters: ";
            for (unsigned i = 0; i < 5 * SVMHandClassifier::NUM_SVMS; ++i) {
                std::cout << hyperparams[i] << " ";
            }
            std::cout << "\n";

            hc->train(trainingPath, hyperparams);
            hc->exportFile(savePath);
        }
        else {
            continue;
        }

    }
}
