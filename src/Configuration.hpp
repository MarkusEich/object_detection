#ifndef OBJECT_DETECTION_CONFIGURATION_HPP__
#define OBJECT_DETECTION_CONFIGURATION_HPP__

namespace object_detection
{

/** configuration object for object detection */
struct ConfigBlobDetection
{

    int min_area;
    float min_likelihood;
    float scale;

    // read http://code.opencv.org/projects/opencv/wiki/MSER
    // http://www.vlfeat.org/overview/mser.html
    int mser_delta;				// stability criteria
    int mser_max_area;			// max area of segmented blob that should be tracked
    float mser_max_variation; 	
    float mser_min_diversity;	// if the area difference between overlaid blobs are within min_diversity
    							// than just one of them will be selected as stable region
    							// criteria to prune similar blobs


    ConfigBlobDetection() :
        min_area(100),
        min_likelihood(0.5),
        scale(1.0),
        mser_delta(4),
        mser_max_area(8400),
        mser_max_variation(0.15),
        mser_min_diversity(0.5)
    {}

    // TODO add configuration values here

};

}

#endif
