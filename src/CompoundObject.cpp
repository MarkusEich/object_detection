/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#include "CompoundObject.hpp"

using namespace object_detection;

template<class TComponent>
CompoundObject<TComponent>::CompoundObject():assembled_(false),modelSet_(false),use_color_(false)
{    
    pose_.position=base::Position(0,0,0);
    pose_.orientation=base::Orientation(1,0,0,0);

    dimensions_=base::Vector3d(0,0,0);
    likelihood_=0;
    debug_=true;

}

template<class TComponent>
void CompoundObject<TComponent>::addComponent(TComponent component){

    components_.push_back(component);

}
template<class TComponent>
base::Vector3d CompoundObject<TComponent>::getEuler(){

    base::Vector3d euler;
    base::Matrix3d m;
    m=pose_.orientation.toRotationMatrix();

    //fill roll pitch yaw
    euler[0]=atan2(m(3,1),m(3,2));
    euler[1]=acos(m(2,2));
    euler[2]=-atan2(m(1,3),m(2,3));

    return euler;

}

//instanciate because template is outside header

template class CompoundObject<object_detection::Plane>;
template class CompoundObject<object_detection::Hole>;
