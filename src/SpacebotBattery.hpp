#ifndef SPACEBOTBATTERY_HPP
#define SPACEBOTBATTERY_HPP

#include <object_detection/CompoundObject.hpp>
#include <object_detection/Objects.hpp>
#include <object_detection/Plane.hpp>
#include <object_detection/Hole.hpp>


namespace object_detection{

class SpacebotBattery : public CompoundObject<Plane>
{
public:
    SpacebotBattery();


    void setModel(const Box& boxModel){
        boxModel_.color=boxModel.color;
        boxModel_.dimensions=boxModel.dimensions;
        this->modelSet_=true;
        dimensions_=boxModel_.dimensions;
    }


    bool assembleObject();


private:

    //mode for the box. Used to assemble the box based on the components
    Box boxModel_;//

    void setBatteryPose(Plane plane,BOX_PANEL_SIDE side);


};
}

#endif // SPACEBOTBATTERY_HPP
