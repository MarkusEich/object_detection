#ifndef OBJECT_DETECTION_BLOB_DETECTION_HPP__
#define OBJECT_DETECTION_BLOB_DETECTION_HPP__

#include <object_detection/Configuration.hpp>
#include <object_detection/Objects.hpp>
#include <object_detection/Blob.hpp>
#include <opencv2/opencv.hpp>

#include <boost/math/distributions/normal.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace object_detection
{

class BlobDetection
{ 
    ConfigBlobDetection config;
    std::vector<Object::Ptr> objList; 
    std::vector<std::vector<Blob> > candidates;

    cv::Mat dbg_img;
    cv::Mat mask;

public: 
    /**
     * set the configuration for the blob detection
     */
    void setConfiguration( const ConfigBlobDetection& config );

    void setMaskImage( const std::string& img_path );

    /**
     * set the objects which should be detected
     *
     * note: this is quite focused detection of 3d objects with a known size
     * and color. There is no reason not to include some sort of intermediate
     * representation of what we are actually looking for.
     */
    void setObjectList( const std::vector<PrimitiveObject> &list );

    /**
     * Process the image frame, and try to find the objects in this frame.
     *
     * The result is returned using the getCandidates method
     *
     */
    void processImage( const cv::Mat &frame );

    /** 
     * generate a debug image, which shows the internal state of the processing
     * in a human understandable format
     */
    cv::Mat getDebugImage();

    /**
     * @brief return the number of candidate blobs which have been found in the image
     *
     * Vector will be empty if no adequate blobs have been found, or if
     * processImage() has not been called yet.
     *
     * @return a vector of vector of candidate blobs that have been found
     */
    std::vector<std::vector<Blob> > getCandidates() const;

private:

    /**
     * @brief calculate euclidean distance between the color of image pixel and target color
     * @param src - source image with the depth CV_32F and three channels
     * @param dst - destination image with the depth CV_32F and one channel
     * @param target_color
     */
    void calcEuclideanDistance(const cv::Mat &src, cv::Mat &dst, const Color &target_color);

    /**
     * @brief calculate the mean of color distance for blob points
     * @param img_scalar - source image with the depth CV_32F and one channel
     * @param blob - the points of the blob
     * @return the mean value of color distance
     */
    float calcMeanInsideBlob(const cv::Mat &img_scalar, const std::vector<cv::Point> &blob );

    /**
     * @brief calculate the standard deviation of color distance for blob points
     * @param img_scalar - source image with the depth CV_32F and one channel
     * @param blob - the points of the blob
     * @param mean_distance - the mean value of color distance
     * @return the standard deviation
     */
    float calcStdDevInsideBlob(const cv::Mat &img_scalar, const std::vector<cv::Point> &blob, float mean_distance);

    void grayWorldNormalization(const cv::Mat &src, cv::Mat &dst);

    /**
     *  @brief calcualte achromatic mask
     *  take float nRGB image and the pixels where the difference between channels is less than thresh
     *  are marked as achromatic
     */
    void calcAchromaticMask(const cv::Mat src, cv::Mat &dst, float thresh);

    /**
     * @brief convert image form rgb color space into normalized rgb
     * can be used for calculate bgr2nbgr
     * @param src - source image with the depth CV_8U and three channels
     * @param dst - destination image with the depth CV_32F and three channels, the value range is [0, 1/3]
     */
    static void cvtRGB2NRGB(const cv::Mat &src, cv::Mat &dst);       
};

} // end namespace object_detection

#endif 
