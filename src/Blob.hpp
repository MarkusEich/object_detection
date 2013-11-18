#ifndef OBJECT_DETECTION_BLOB_HPP__
#define OBJECT_DETECTION_BLOB_HPP__

#include "Time.hpp"
#include "Eigen.hpp"
#include <vector>

namespace object_detection
{
    struct Blob
    {
        /**
         * diameter of the blob normalized for distance.
         *
         * So that for distance to blob in m:
         * actual_diameter = diameter * distance
         */
        float diameter;

        /**
         * value between 0 and 1, where 0 is no confidence, and 1 is absolutely sure
         */
        float confidence;

        /**
         * the center coordinates of blob in the image
         */
        int x_pos_center;
        int y_pos_center;

	/**
	 * Intersection of the direction vector of the point and the plane at
	 * 1m distance.
	 *
	 * Note that the coordinates are still on the image plane, where x is
	 * to the right and y is down.
	 * 
	 * This direction vector can be directly used to trace the projection
	 * line of the detected point, without the need for the intrinsic
	 * calibration.
	 */
	base::Vector2d view_intersection;
    };

    struct Candidates
    {
        std::vector<std::vector<Blob> > blobs;
        base::Time time;
    };
}

#endif
