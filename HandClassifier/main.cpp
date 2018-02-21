#include "HandClassifier.h"
#include "core.h"

using namespace ark::classifier;

const char * trainingPath = "D:\\Programming\\3VR\\ht_tests_v3\\train\\";
const std::string benchmarkPaths[] = { "D:\\Programming\\3VR\\ht_tests_v3\\test\\",  "D:\\Programming\\3VR\\ht_tests_v3\\train\\", 
"D:\\Programming\\3VR\\ht_tests_v2\\test\\", "D:\\Programming\\3VR\\ht_tests_v2\\train\\"};
const char * savePath = "D:\\Programming\\3VR\\OpenARK\\svm\\";

int main(void) {
    int ihyperparams[5];

    std::string names[] = { "gamma", "coef0", "C", "eps", "p" };

    cv::namedWindow("SVM");
    for (unsigned i = 0; i < 5; ++i) {
        std::string windowName = "SVM";

        if (i == 3) {
            ihyperparams[i] = (int)(SVMHandValidator::DEFAULT_HYPERPARAMS[i] * 1e17);
            cv::createTrackbar(names[i], windowName, &ihyperparams[i], ihyperparams[i] * 4);
        }
        else {
            ihyperparams[i] = (int)(SVMHandValidator::DEFAULT_HYPERPARAMS[i] * 10000.0);
            if (i % 5 == 0) 
                cv::createTrackbar(names[i], windowName, &ihyperparams[i], 100000.0);
            else
                cv::createTrackbar(names[i], windowName, &ihyperparams[i], 10000.0);
        }

    }

    int boundary = 500;
    cv::createTrackbar("Boundary", "SVM", &boundary, 1000);

    double hyperparams[5];

    boost::shared_ptr<HandClassifier> hc = boost::make_shared<SVMHandValidator>(savePath);

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

            int numFalseNeg = 0, numFalsePos = 0, total = 0;
            for (int i = 0; i < n; ++i) {
                std::string name; int nFeat;

                if (!(ifsFeat >> name >> nFeat)) break;
                cv::Mat feats (1, nFeat, CV_32F);

                float * ptr = feats.ptr<float>(0);
                for (int i = 0; i < nFeat; ++i) {
                    ifsFeat >> ptr[i];
                }

                std::string lab = ""; int correct;
                while (lab != name && (ifs >> lab >> correct)) {};

                ++total;
                float result = hc->classify(feats);

                if (result < (boundary / 1000.0)) {
                    ++zeros;
                }

                double confidence = std::min(abs(result - boundary / 1000.0) / abs(correct - boundary / 1000.0), 1.0);

                int prediction = (result >= boundary / 1000.0 ? 1 : 0);

                if (prediction == correct) {
                    ++totalCorrect;
                }
                else {
                    std::cout << "#" << lab << ": predicted " << prediction << " with confidence " <<
                        confidence << " but correct label is " << correct << "\n";

                    if (prediction == 1) {
                        ++numFalsePos;
                    }
                    else {
                        ++numFalseNeg;
                    }
                }
            }

            std::cout << "\n" << std::endl;
            std::cout << totalCorrect / total * 100.0 << "% Correctly Predicted.\n";
            std::cout << zeros / total * 100.0 << "% of predictions are zeros\n\n";
            std::cout << numFalsePos << " False Positives, " << numFalseNeg << " False Negatives." << std::endl;

            ifs.close();
            ifsFeat.close();
        }
        else if (c == 't') {
            for (unsigned i = 0; i < 5; ++i) {
                if (i == 3)
                    hyperparams[i] = (double)ihyperparams[i] / 1e17;
                else
                    hyperparams[i] = (double)ihyperparams[i] / 10000;
            }

            std::cout << "Training with hyperparameters: ";
            for (unsigned i = 0; i < 5; ++i) {
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
