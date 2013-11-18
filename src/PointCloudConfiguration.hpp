#ifndef POINTCLOUDDETECTION_CONFIGURATION__
#define POINTCLOUDDETECTION_CONFIGURATION__

namespace object_detection
{
namespace point_cloud
{

struct Configuration
{
    Configuration() :
	smoothness( 7.0 ),
	curvature( 1.0 ),
	cylinderModelThreshold( 0.7 ),
	boxModelThreshold( 0.7 ),
	groundDistance( 0.30 )
    {}

    /** @brief Sets params for the plane region growing
      * @curvature max curvature of plane
      * @smoothness max deviance betweeen two normals of neighbors
      */
    void inline setPlaneParams(const double& curvature,const double& smoothness){this->curvature=curvature;this->smoothness=smoothness;}
    void inline getPlaneParams(double& curvature,double& smoothness){curvature=this->curvature;smoothness=this->smoothness;}

    /*getter setter for model thresholds*/
    void inline setCylinderModelThreshold(const double& cylinderModelThreshold){this->cylinderModelThreshold=cylinderModelThreshold;}
    void inline setBoxModelThreshold(const double& boxModelThreshold){this->boxModelThreshold=boxModelThreshold;}
    double inline getCylinderModelThreshold(){return cylinderModelThreshold;}
    double inline getBoxModelThreshold(){return boxModelThreshold;}

    void setExpectedGroundDistance(double ground_distance){groundDistance=ground_distance;}

    double smoothness;
    double curvature;
    double cylinderModelThreshold;
    double boxModelThreshold;
    double groundDistance;
};

}
}

#endif
