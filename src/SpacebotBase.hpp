/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef SPACEBOTBASE_HPP
#define SPACEBOTBASE_HPP


#include <object_detection/CompoundObject.hpp>
#include <object_detection/Objects.hpp>
#include <object_detection/Plane.hpp>
#include <object_detection/Hole.hpp>



namespace object_detection
{



class SpacebotBase: public CompoundObject<Plane>
{
public:
    SpacebotBase();

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
    void setBasePose(Plane plane,BOX_PANEL_SIDE side);


};



}

#endif // SPACEBOTBASE_HPP
