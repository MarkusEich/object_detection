/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef HOLE_HPP
#define HOLE_HPP

#include<object_detection/Component.hpp>

namespace object_detection{

class Hole : public Component
{
public:
    Hole(double area, double width,double height, base::Pose);


    Hole(const Hole& hole):Component(hole)
    {
        area_=hole.area_;
        width_=hole.width_;
        height_=hole.height_;
        pose_=hole.pose_;

    }


private:

    double area_;
    double width_;
    double height_;    
};
}

#endif // HOLE_HPP
