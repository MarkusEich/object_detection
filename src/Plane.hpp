/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef PLANE_HPP
#define PLANE_HPP

#include <object_detection/Component.hpp>

#include <segmentation/plane.h>

namespace object_detection{


class Plane : public Component
{
public:
    Plane(double rectangularness, double area, double width,double height, base::Pose pose,base::Vector3d normal,base::Vector3d expansion,double aboveGround=0.0);
    Plane(const Plane& plane):Component(plane)
    {
        rectangularness_=plane.rectangularness_;
        area_=plane.area_;
        width_=plane.width_;
        height_=plane.height_;
        pose_=plane.pose_;        
        normal_=plane.normal_;
        aboveGround_=plane.aboveGround_;
        expansion_=plane.expansion_;

    }

    double isParallel(object_detection::Plane& otherPlane);
    double isOrthogonal(object_detection::Plane& otherPlane);
    double getCogDistance(object_detection::Plane& otherPlane);
    double matchesShape(double rectangularness,double width,double height);   
    bool hasCommonEdge(object_detection::Plane& otherPlane);    

    double isAboveGround(){return aboveGround_;}
    base::Vector3d getNormal(){return normal_;}
    base::Vector3d getExpansion(){return expansion_;}

    bool hasSlot(double width,double height);

    double getWidth(){return width_;}
    double getHeight(){return height_;}




private:

    base::Vector3d normal_;
    base::Vector3d expansion_; //spatial expansion in world coordinates
    double rectangularness_;
    double area_;
    double width_;
    double height_;        
    double aboveGround_;


};

}

#endif // PLANE_HPP
