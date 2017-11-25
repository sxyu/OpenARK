#pragma once

// OpenCV Libraries
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

//OpenARK libraries
#include "Util.h"
#include "Hand.h"
#include "Object3D.h"

/**
 * Class defining general behavior of a depth camera.
 * Any depth camera should be able to generate a XYZMap, AmpMap (confidence), and FlagMap.
 */
class DepthCamera
{
public:
    virtual ~DepthCamera();

    /**
     * Retrieve the next frame from the depth camera
     * @param removeNoise if true, performs noise removal on the depth image after retrieving it
     */
    void nextFrame(bool removeNoise = true);

    /**
     * Closes and exists the depth camera.
     * This function should be overriden with a concrete implementation depending on the specific depth camera
     */
    virtual void destroyInstance() =0;

    /**
     * @Deprecated
     * @deprecated please use queryObjects(), queryHands(), etc. instead
     * Performs euclidean clustering to separate discrete objects in the input point cloud.
     * @param max_distance the maximum allowed distance to be clustered
     * @param min_points the minimum number of points a valid cluster should have
     * @param min_size the minimum surface area (in m^2) a valid cluster should have
     * @param max_size the maximum surface area (in m^2) a valid cluster should have
     * @param floodfill_interval the x, y interval between points at which we should try to flood fill. higher = faster 
     */
    void computeClusters(double max_distance = 0.007, double max_ir_distance = 150,
        int min_points = 1900, double min_size = 0.008, double max_size = 0.055,
        int dilate_amount = 2, int floodfill_interval = 10);

    /**
     * Reads a sample frame from file.
     * @param source the directory which the frame file is stored
     */
    bool readImage(std::string source);

    /**
     * Writes the current frame into file.
     * @param destination the directory which the frame should be written to
     */
    bool writeImage(std::string destination) const;

    /**
     * Removes noise from the XYZMap based on confidence provided in the AmpMap and FlagMap.
     */
    void removeNoise();

    ///**
    // * Removes all points defined by the coordinates in the points vector from the XYZMap.
    // * @param [in] points the list of coordinates where data from the XYZMap should be removed
    // */
    //void removePoints(std::vector<cv::Point2i> points);

    /**
     * Returns the current XYZMap.
     */
    const cv::Mat getXYZMap() const;

    /**
     * Returns the current AmpMap
     */
    const cv::Mat getAmpMap() const;

    /**
     * Returns the current FlagMap.
     */
    const cv::Mat getFlagMap() const;

    /**
     * Returns the width of the frame in pixels.
     */
    int getWidth() const;

    /**
     * Returns the height of the frame in pixels.
     */
    int getHeight() const;

    /**
     * Returns true if an RGB image is available from this camera.
     */
    virtual bool hasRGBImage() const;

    /**
     * Get the RGB Image from this camera, if available. Else, throws an error.
     */
    virtual const cv::Mat & getRGBImage();

    /**
     * Returns true if an RGB image is available from this camera.
     */
    virtual bool hasIRImage() const;

    /**
     * Get the infrared (IR) Image from this camera, if available. Else, throws an error.
     */
    virtual const cv::Mat & getIRImage();

    /**
     * @Deprecated
     * @deprecated please use queryObjects(), queryHands(), etc. instead
     * Returns all the clusters (discrete objects) in the current frame. 
     * Note: Must run computeClusters() before calling this function.
     * @see computeClusters
     * @see getClusterAreas
     * @return vector of matrixes with each matrix corresponding to a cluster in no particular order
     */
    std::vector<cv::Mat> getClusters() const;

    /**
     * @Deprecated
     * @deprecated please use queryObjects(), queryHands(), etc. instead
     * Returns the surface areas of the clusters (discrete objects) in the current frame. 
     * Note: Must run computeClusters() before calling this function.
     * @see computeClusters
     * @see getClusters
     * @return vector containing surface areas of clusters, in meters squared.
     */
    std::vector<double> getClusterAreas() const;

    /*
     * Retrieve a list of objects visible in the current frame
     * @see queryHands
     * @param max_distance the maximum allowed distance to be clustered
     * @param min_points the minimum number of points a valid cluster should have
     * @param min_size the minimum surface area (in m^2) a valid cluster should have
     * @param max_size the maximum surface area (in m^2) a valid cluster should have
     * @param floodfill_interval the x, y interval between points at which we should try to flood fill. higher = faster 
     * @return vector of visible objects
     */
    std::vector<Object3D> queryObjects(double max_distance = 0.007, double max_ir_distance = 150,
        int min_points = 1900, double min_size = 0.008, double max_size = 0.055,
        int dilate_amount = 2, int floodfill_interval = 10);

    /*
     * Retrieve a list of hands visible in the current frame
     * @see queryObjects
     * @param max_distance the maximum allowed distance to be clustered
     * @param min_points the minimum number of points a valid cluster should have
     * @param min_size the minimum surface area (in m^2) a valid cluster should have
     * @param max_size the maximum surface area (in m^2) a valid cluster should have
     * @param floodfill_interval the x, y interval between points at which we should try to flood fill. higher = faster 
     * @return vector of visible hands
     */
    std::vector<Hand> queryHands(double max_distance = 0.007, double max_ir_distance = 150,
        int min_points = 1900, double min_size = 0.008, double max_size = 0.055,
        int dilate_amount = 2, int floodfill_interval = 10);

    bool badInput;

protected:
    /**
     * Retrieve the next frame from the camera
     * This function should be overriden with a concrete implementation depending on the specific depth camera
     */
    virtual void update() =0;

    /**
     * Initializes all variables used by the generic depth camera.
     * Sets xyzMap, ampMap, flagMap, and clusters to empty
     */
    void initializeImages();

    /**
     * Performs floodfill starting from seed point (x,y).
     * @param seed_x x-coordinate of the seed point
     * @param seed_y y-coordinate of the seed point
     * @param [in] zMap the xyzMap point cloud
     * @param max_distance the maximum euclidean distance allowed between neighbors
     * @param output_points optionally, pointer to a vector in which we will 
              store the points in the component. This vector should be AT LEAST the size of the xyz map
     * @param [in] irImg optionally, the infrared image corresponding to the point cloud
     * @param ir_distance the maximum difference in IR distance between neighbors (only used if irImg specified)
     * @param [out] mask optionally, output image containing points visited by the floodfill
     * @returns number of points in component
     */
    static int floodFill(int seed_x, int seed_y, cv::Mat& zMap,
       std::vector <cv::Point> * output_points = nullptr, double max_distance = 0.04,
        cv::Mat& irImg = cv::Mat(), double ir_distance = 25, cv::Mat& mask = cv::Mat());

    ///**
    // * Analyze the candidate point with its neighbors to determine whether they belong to the same cluster.
    // * @param x x-coordinate of the candidate point
    // * @param y y-coordinate of the candidate point
    // * @param [in] depthMap the xyzMap point cloud of the scene
    // * @param num_neighbors the number of neighbors to consider
    // * @param max_distance the maximum euclidean distance allowed between neighbors
    // */
    //static bool closeEnough(int x, int y, cv::Mat& depthMap, int num_neighbors, double max_distance);

    /**
     * Stores the (x,y,z) data of every point in the observable world.
     * Matrix type CV_32FC3
     */
    cv::Mat xyzMap;

    /**
     * Stores the confidence value of each corresponding point in the world.
     * Matrix type CV_32FC1
     */
    cv::Mat ampMap;

    /**
     * Stores additional information about the points in the world.
     * Matrix type CV_8UC1
     */
    cv::Mat flagMap;

    /**
     * Stores the RGB image from this camera, if available
     * Matrix type CV_8UC3
     */
    cv::Mat rgbImage;

    /**
     * Stores the infrared image from this camera, if available
     * Matrix type CV_16U
     */
    cv::Mat irImage;

    /**
     * Stores the each individual cluster in its individual XYZMap. (Will be deleted)
     */
    std::vector<cv::Mat> clusters;

    /**
     * Stores the surface area of each cluster (Will be deleted)
     */
    std::vector<double> clusterAreas;


    /**
     * Stores the objects visible to the camera
     */
    std::vector<Object3D> objects;

    /**
     * Value that determines the validity of a point in respect to the ampMap.
     * This value varies from sensor to sensor so it should be define when constructing child class
     */
    double CONFIDENCE_THRESHHOLD;

    /**
     * Value that determines the validity of a point in respect to the flagMap.
     * This value varies from sensor to sensor so it should be defined when constructing child class
     */
    int INVALID_FLAG_VALUE;

    /**
     * The image width resolution (pixels) that the depth sensor produces.
     */
    int X_DIMENSION = 321;


    /**
     * The image height resolution (pixels) that the depth sensor produces.
     */
    int Y_DIMENSION = 240;
};