/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#include "Hole.hpp"

using namespace object_detection;

Hole::Hole(double area, double width, double height, base::Pose pose):Component(pose)
{
    area_=area;
    width_=width;
    height_=height;    
    pose_=pose;
}
