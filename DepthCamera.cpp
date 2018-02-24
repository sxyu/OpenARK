#include "stdafx.h"
#include "version.h"
#include "DepthCamera.h"
#include "Hand.h"
#include "FrameObject.h"

namespace ark {

    /**
     * Minimum depth of points (in meters). Points under this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_LOW = 0.14;

    /**
     * Maximum depth of points (in meters). Points above this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_HIGH = 0.99;

    DepthCamera::~DepthCamera()
    {
        badInputFlag = true;
        endCapture();
    }

    void DepthCamera::beginCapture(int fps_cap, bool remove_noise)
    {
        ASSERT(captureInterrupt == true, "beginCapture: already capturing from this camera");
        captureInterrupt = false;
        std::thread thd(&DepthCamera::captureThreadingHelper, this, fps_cap,
            &captureInterrupt, remove_noise);
        thd.detach();
    }

    void DepthCamera::endCapture()
    {
        captureInterrupt = true;
    }

    bool DepthCamera::nextFrame(bool removeNoise)
    {
        // initialize back buffers
        initializeImages();

        // call update with back buffer images (to allow continued operation on front end)
        update(xyzMapBuf, rgbMapBuf, irMapBuf, ampMapBuf, flagMapBuf);

        if (!badInput() && xyzMapBuf.data) {
            if (removeNoise) {
                this->removeNoise(xyzMapBuf, ampMapBuf, flagMapConfidenceThreshold());
            }
        }

        // lock all buffers while swapping
        std::lock_guard<std::mutex> lock(imageMutex);

        // when update is done, swap buffers to front
        swapBuffers();

        // clear cache flag
        isCached = 0;
        return !badInput();
    }

    void DepthCamera::computeNormalMap(const ObjectParams * params, const cv::Mat * xyz_map) {
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        if (xyz_map) {
            util::computeNormalMap(*xyz_map, normalMap, 4, params->normalResolution, false);
        }
        else {
            util::computeNormalMap(this->xyzMap, normalMap, 4, params->normalResolution, false);
        }

        isCached |= FLAG_NORMALS;
    }

    std::vector<HandPtr> & DepthCamera::getFrameHands(const ObjectParams * params, bool elim_planes)
    {
        if (!this->xyzMap.data || (isCached & FLAG_FRM_HANDS)) return hands;

        hands.clear();

        // 1. initialize
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        cv::Mat xyzMap = getXYZMap();
        const int R = xyzMap.rows, C = xyzMap.cols;

        cv::Mat floodFillMap(R, C, CV_8U);

        const Vec3f * ptr;
        uchar * visPtr;

        for (int r = 0; r < R; ++r)
        {
            visPtr = floodFillMap.ptr<uchar>(r);
            ptr = xyzMap.ptr<Vec3f>(r);
            for (int c = 0; c < C; ++c)
            {
                visPtr[c] = ptr[c][2] > 0 ? 255 : 0;
            }
        }

        // 2. eliminate large planes

        if (elim_planes){
            std::vector<FramePlanePtr> planes = getFramePlanes(params);
            if (planes.size()) {
                for (FramePlanePtr plane : planes) {
                    util::removePlane<uchar>(xyzMap, floodFillMap, plane->equation,
                        params->handPlaneMinNorm);
                }
            }
        }

        // 3. flood fill on point cloud 
        boost::shared_ptr<Hand> bestHandObject;
        float closestHandDist = FLT_MAX;

        std::vector<Point2i> allIjPoints(R * C);
        std::vector<Vec3f> allXyzPoints(R * C);

        // compute the minimum number of points in a cluster according to params
        const int CLUSTER_MIN_POINTS = (int) (params->handClusterMinPoints * R * C);

        for (int r = 0; r < R; r += params->handClusterInterval)
        {
            ptr = xyzMap.ptr<Vec3f>(r);
            visPtr = floodFillMap.ptr<uchar>(r);

            for (int c = 0; c < C; c += params->handClusterInterval)
            {
                if (visPtr[c] > 0 && ptr[c][2] > 0)
                {
                    int points_in_comp = util::floodFill(xyzMap, Point2i(c, r),
                        params->handClusterMaxDistance,
                        &allIjPoints, &allXyzPoints, nullptr, 1, 4,
                        params->handClusterMaxDistance * 12, &floodFillMap);

                    if (points_in_comp >= CLUSTER_MIN_POINTS)
                    {
                        auto ijPoints = boost::make_shared<std::vector<Point2i> >
                            (allIjPoints.begin(), allIjPoints.begin() + points_in_comp);
                        auto xyzPoints = boost::make_shared<std::vector<Vec3f> >
                            (allXyzPoints.begin(), allXyzPoints.begin() + points_in_comp);

                        // 4. for each cluster, test if hand

                        // if matching required conditions, construct 3D object
                        auto handPtr = boost::make_shared<Hand>(ijPoints, xyzPoints, xyzMap,
                                params, false, points_in_comp);

                        if (handPtr->isValidHand()) {
                            float distance = handPtr->getDepth();

                            if (distance < closestHandDist) {
                                bestHandObject = handPtr;
                                closestHandDist = distance;
                            }

                            if (handPtr->getSVMConfidence() >
                                params->handSVMHighConfidenceThresh ||
                                !params->handUseSVM) {
                                 // avoid duplicate hand
                                if (bestHandObject == handPtr) bestHandObject = nullptr;
                                hands.push_back(handPtr);
                            }
                        }
                    }
                }
            }
        }

        if (bestHandObject != nullptr) {
            // if no hands surpass 'high confidence threshold', at least add one hand
            hands.push_back(bestHandObject);
        }

        isCached |= FLAG_FRM_HANDS;
        return hands;
    }

    std::vector<FramePlanePtr> & DepthCamera::getFramePlanes(const ObjectParams * params)
    {
        if (!this->xyzMap.data || (isCached & FLAG_FRM_PLANES)) return framePlanes;

        framePlanes.clear();
        cv::Mat xyzMap = getXYZMap();

        std::vector<Vec3f> equations;
        std::vector<VecP2iPtr> points;
        std::vector<VecV3fPtr> pointsXYZ;

        detectPlaneHelper(xyzMap, equations, points, pointsXYZ, params);

        for (uint i = 0; i < equations.size(); ++i) {
            auto planePtr = boost::make_shared<FramePlane>
                (equations[i], points[i], pointsXYZ[i], xyzMap, params);

            if (planePtr->getSurfArea() > params->planeMinArea) {
                framePlanes.emplace_back(planePtr);
            }
        }

        if (params == nullptr) params = &ObjectParams::DEFAULT;

#ifdef DEBUG
        cv::Mat planeDebugVisual =
            cv::Mat::zeros(xyzMap.size() / params->normalResolution, CV_8UC3);

        for (int i = 0; i < framePlanes.size(); ++i) {
            Vec3b color = util::paletteColor(i);
            const std::vector<Point2i> & points = framePlanes[i]->getPointsIJ();

            for (uint j = 0; j < points.size(); ++j) {
                planeDebugVisual.at<Vec3b>(points[j] / params->normalResolution) = color;
            }
        }

        cv::resize(planeDebugVisual, planeDebugVisual, planeDebugVisual.size() * params->normalResolution,
            0, 0, cv::INTER_NEAREST);


        for (int i = 0; i < framePlanes.size(); ++i) {
            cv::putText(planeDebugVisual, std::to_string(framePlanes[i]->getSurfArea()), framePlanes[i]->getCenterIJ(), 0, 0.5, cv::Scalar(255, 255, 255));
        }

        cv::imshow("[Plane Debug]", planeDebugVisual);
#endif

        // cache planes for current frame
        isCached |= FLAG_FRM_PLANES;
        return framePlanes;
    }

    std::vector<FrameObjectPtr> & DepthCamera::getFrameObjects(const ObjectParams * params)
    {        
        if (!this->xyzMap.data || (isCached & FLAG_FRM_OBJS)) return frameObjects;
        frameObjects.clear();

        // default parameters
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        // get planes and hands
        getFramePlanes(params); getFrameHands(params);

        // push resulting objects
        for (auto plane : framePlanes) frameObjects.push_back(plane);
        for (auto hand : hands) frameObjects.push_back(hand);

        // cache for current frame
        isCached |= FLAG_FRM_OBJS;
        return frameObjects;
    }

    bool DepthCamera::badInput()
    {
        return badInputFlag;
    }

    /**
    Remove noise on zMap and xyzMap
    */
    void DepthCamera::removeNoise(cv::Mat & xyz_map, cv::Mat & amp_map, float confidence_thresh)
    {
        for (int r = 0; r < xyz_map.rows; ++r)
        {
            Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

            const float * ampptr = nullptr;
            if (amp_map.data) ampptr = amp_map.ptr<float>(r);

            for (int c = 0; c < xyz_map.cols; ++c)
            {
                if (ptr[c][2] > 0.0f) {
                    if (ptr[c][2] < NOISE_FILTER_LOW &&
                        (ptr[c][2] > NOISE_FILTER_HIGH || ptr[c][2] == 0.0) &&
                        (ampptr == nullptr || amp_map.data == nullptr ||
                            ampptr[c] < confidence_thresh)) {
                        ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                    }
                }
            }
        }
    }

    bool DepthCamera::isCapturing()
    {
        return !captureInterrupt;
    }

    cv::Size DepthCamera::getImageSize() const
    {
        return cv::Size(getWidth(), getHeight());
    }


    const std::string DepthCamera::getModelName() const {
        return "DepthCamera";
    }

    void DepthCamera::initializeImages()
    {
        cv::Size sz = getImageSize();

        // initialize back buffers, if necessary
        xyzMapBuf.release();
        xyzMapBuf.create(sz, CV_32FC3);

        if (hasRGBMap()) {
            rgbMapBuf.release();
            rgbMapBuf.create(sz, CV_8UC3);
        }

        if (hasIRMap()) {
            irMapBuf.release();
            irMapBuf.create(sz, CV_8U);
        }

        if (hasAmpMap()) {
            ampMapBuf.release();
            ampMapBuf.create(sz, CV_32F);
        }

        if (hasFlagMap()) {
            flagMapBuf.release();
            flagMapBuf.create(sz, CV_8U);
        }
    }

    /** swap a single buffer */
    void DepthCamera::swapBuffer(bool (DepthCamera::* check_func)() const, cv::Mat & img, cv::Mat & buf)
    {
        if ((this->*check_func)()) {
            cv::swap(img, buf);
        }
        else {
            img.data = nullptr;
        }
    }

    /** swap all buffers */
    void DepthCamera::swapBuffers()
    {
        cv::swap(xyzMap, xyzMapBuf);
        swapBuffer(&DepthCamera::hasRGBMap, rgbMap, rgbMapBuf);
        swapBuffer(&DepthCamera::hasIRMap, irMap, irMapBuf);
        swapBuffer(&DepthCamera::hasAmpMap, ampMap, ampMapBuf);
        swapBuffer(&DepthCamera::hasFlagMap, flagMap, flagMapBuf);
    }

    /**
    write a frame into file located at "destination"
    */
    bool DepthCamera::writeImage(std::string destination) const
    {
        cv::FileStorage fs(destination, cv::FileStorage::WRITE);
        std::lock_guard<std::mutex> lock(imageMutex);

        fs << "xyzMap" << xyzMap;
        fs << "ampMap" << ampMap;
        fs << "flagMap" << flagMap;
        fs << "rgbMap" << rgbMap;
        fs << "irMap" << irMap;

        fs.release();
        return true;
    }

    /**
    Reads a frame from file located at "source"
    */
    bool DepthCamera::readImage(std::string source)
    {
        cv::FileStorage fs;
        fs.open(source, cv::FileStorage::READ);

        std::lock_guard<std::mutex> lock(imageMutex);

        fs["xyzMap"] >> xyzMap;
        fs["ampMap"] >> ampMap;
        fs["flagMap"] >> flagMap;
        fs["rgbMap"] >> rgbMap;
        fs["irMap"] >> irMap;
        fs.release();

        isCached = 0;

        if (xyzMap.rows == 0 || ampMap.rows == 0 || flagMap.rows == 0)
        {
            return false;
        }

        return true;
    }

    const cv::Mat DepthCamera::getXYZMap() const
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (xyzMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32FC3);
        return xyzMap;
    }

    const cv::Mat DepthCamera::getAmpMap() const
    {
        if (!hasAmpMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (ampMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32F);
        return ampMap;
    }

    const cv::Mat DepthCamera::getFlagMap() const
    {
        if (!hasFlagMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (flagMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8U);
        return flagMap;
    }

    const cv::Mat DepthCamera::getRGBMap() const {
        if (!hasRGBMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (rgbMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8UC3);
        return rgbMap;
    }

    const cv::Mat DepthCamera::getIRMap() const
    {
        if (!hasIRMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (irMap.data == nullptr) return cv::Mat::zeros(getImageSize(), CV_8U);
        return irMap;
    }

    const cv::Mat DepthCamera::getNormalMap()
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if ((!(isCached & FLAG_NORMALS) || normalMap.data == nullptr) &&
                            xyzMap.data != nullptr){
            computeNormalMap();
        }

        return normalMap;
    }

    bool DepthCamera::hasAmpMap() const
    {
        // Assume no amp map, unless overridden
        return false;
    }

    bool DepthCamera::hasFlagMap() const
    {
        // Assume no flag map, unless overridden
        return false;
    }

    bool DepthCamera::hasRGBMap() const {
        // Assume no RGB image, unless overridden
        return false;
    }

    bool DepthCamera::hasIRMap() const
    {
        // Assume no IR image, unless overridden
        return false;
    }

    // note: depth camera must have XYZ map

    int DepthCamera::ampMapInvalidFlagValue() const{
        return -1;
    }

    float DepthCamera::flagMapConfidenceThreshold() const{
        return 0.5;
    }

    void DepthCamera::captureThreadingHelper(int fps_cap, volatile bool * interrupt, bool remove_noise)
    {
        using namespace std::chrono;
        steady_clock::time_point lastTime;
        steady_clock::time_point currTime;
        float timePerFrame;
        if (fps_cap > 0) {
            timePerFrame = 1e9f / fps_cap;
            lastTime = steady_clock::now();
        }

        while (interrupt == nullptr || !(*interrupt)) {
            this->nextFrame(remove_noise);

            // cap FPS
            if (fps_cap > 0) {
                currTime = steady_clock::now();
                steady_clock::duration delta = duration_cast<microseconds>(currTime - lastTime);

                if (delta.count() < timePerFrame) {
                    long long ms = (long long)(timePerFrame - delta.count()) / 1e6f;
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(ms));
                }
                lastTime = currTime;
            }
        }
    }

    void DepthCamera::detectPlaneHelper(const cv::Mat & xyz_map,
        std::vector<Vec3f> & output_equations, 
        std::vector<VecP2iPtr> & output_points, std::vector<VecV3fPtr> & output_points_xyz, 
        const ObjectParams * params, const cv::Mat * normal_map)
    { 
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        // 1. initialize
        cv::Mat normalMap;
        if (normal_map == nullptr) {
            normalMap = getNormalMap();
        }
        else {
            normalMap = *normal_map;
        }
        const int R = xyz_map.rows, C = xyz_map.cols, N = R * C;

        // initialize flood fill map
        cv::Mat floodFillMap(R, C, CV_8U);
        const Vec3f * ptr; uchar * visPtr;
        for (int r = 0; r < R; ++r)
        {
            visPtr = floodFillMap.ptr<uchar>(r);
            ptr = xyz_map.ptr<Vec3f>(r);
            for (int c = 0; c < C; ++c)
            {
                visPtr[c] = ptr[c][2] > 0 ? 255 : 0;
            }
        }

        int compId = -1;
        std::vector<Point2i> allIndices(N);

        // 2. find 'subplanes' i.e. all flat objects visible in frame and combine similar ones
        // stores points on each plane
        std::vector<boost::shared_ptr<std::vector<Point2i> > > planePointsIJ;

        // stores points (in 3D coords) on each plane
        std::vector<boost::shared_ptr<std::vector<Vec3f> > > planePointsXYZ;

        // equations of the planes: ax + by - z + c = 0
        std::vector<Vec3f> planeEquation;

        // compute constants
        const int SUBPLANE_MIN_POINTS = params->subplaneMinPoints * N /
            (params->normalResolution * params->normalResolution);

        const int PLANE_MIN_POINTS = params->planeMinPoints * N /
            (params->normalResolution * params->normalResolution);

        const int PLANE_MIN_INLIERS = params->planeEquationMinInliers * N /
            (params->normalResolution * params->normalResolution);

        for (int r = 0; r < R; r += params->normalResolution) {
            visPtr = floodFillMap.ptr<uchar>(r);

            for (int c = 0; c < C; c += params->normalResolution) {
                if (visPtr[c] == 0) continue;

                Point2i pt(c, r);
                // flood fill normals
                int numPts = util::floodFill(normalMap, pt, params->planeFloodFillThreshold,
                                             &allIndices, nullptr, nullptr,
                                             params->normalResolution, 0, 0.0f, &floodFillMap);

                if (numPts >= SUBPLANE_MIN_POINTS) {
                    // filter out outliers & find plane equation
                    std::vector<Vec3f> allXyzPoints(numPts);

                    for (int k = 0; k < numPts; ++k) {
                        allXyzPoints[k] = xyz_map.at<Vec3f>(allIndices[k]);;
                    }

                    util::radixSortPoints(allIndices, C, R, numPts, &allXyzPoints);
                    double surfArea = util::surfaceArea(normalMap.size(), allIndices,
                        allXyzPoints, numPts);

                    if (surfArea < params->subplaneMinArea) {
                        continue;
                    }

                    Vec3f eqn = util::linearRegression(allXyzPoints, numPts);

                    // combine similar subplanes
                    uint i;
                    for (i = 0; i < planeEquation.size(); ++i) {
                        if (util::norm(planeEquation[i] - eqn) < params->planeCombineThreshold) {
                            // found similar subplane, so combine them
                            break;
                        }
                    }

                    // pointers to point storage in planePointsIJ/XYZ
                    boost::shared_ptr<std::vector<Point2i>> pointStore;
                    boost::shared_ptr<std::vector<Vec3f>> pointStoreXyz;

                    if (i >= planeEquation.size()) {
                        // no similar plane found
                        planeEquation.push_back(eqn);
                        planePointsIJ.emplace_back(boost::make_shared<std::vector<Point2i> >());
                        planePointsXYZ.emplace_back(boost::make_shared<std::vector<Vec3f> >());
                        pointStore = *planePointsIJ.rbegin();
                        pointStoreXyz = *planePointsXYZ.rbegin();
                    }
                    else {
                        // similar plane found
                        pointStore = planePointsIJ[i];
                        pointStoreXyz = planePointsXYZ[i];
                    }

                    // save plane points to store
                    int start = (int)pointStore->size();
                    pointStore->resize(start + numPts);
                    pointStoreXyz->resize(start + numPts);

                    for (int i = 0; i < numPts; ++i) {
                        pointStore->at(start + i) = allIndices[i];
                        pointStoreXyz->at(start + i) = allXyzPoints[i];
                    }
                }
            }
        }

        // 3. find equations of the combined planes and construct Plane objects with the data
        for (uint i = 0; i < planeEquation.size(); ++i) {
            int SZ = (int)planePointsIJ[i]->size();
            if (SZ < PLANE_MIN_POINTS) continue;

            std::vector<Vec3f> pointsXYZ;
            util::removeOutliers(*planePointsXYZ[i], pointsXYZ, params->planeOutlierRemovalThreshold);

            planeEquation[i] = util::linearRegression(pointsXYZ);

            int goodPts = 0;

            for (uint j = 0; j < SZ; ++j) {
                float norm = util::pointPlaneNorm((*planePointsXYZ[i])[j], planeEquation[i]);
                if (norm < params->handPlaneMinNorm) {
                    ++goodPts;
                }
            }

            if (goodPts < PLANE_MIN_INLIERS) continue;

            // push to output
            output_points.push_back(planePointsIJ[i]);
            output_points_xyz.push_back(planePointsXYZ[i]);
            output_equations.push_back(planeEquation[i]);
        }
    }
}
