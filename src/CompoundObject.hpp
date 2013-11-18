/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef COMPOUNTOBJECT_H
#define COMPOUNTOBJECT_H

#include "Plane.hpp"
#include "Hole.hpp"


#include "Pose.hpp"
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

namespace object_detection
{

enum BOX_PANEL_SIDE{
    FRONT_REAR=0,
    SIDE,
    TOP
};

template <class TComponent>
class CompoundObject
{
public:

    CompoundObject();

    void clear(){
        this->deleteComponents();
        this->deletePoints();
    }

    void addComponent(TComponent component);
    TComponent getComponentAt(size_t a){return components_.at(a);}

    inline base::Pose getPose(){return pose_;}
    inline base::Vector3d getPosition(){return pose_.position;}
    inline base::Quaterniond getOrientation(){return pose_.orientation;}
    base::Vector3d getEuler();
    inline base::Vector3d getDimensions(){return dimensions_;}
    virtual bool assembleObject(){return false;}
    bool isAssembled(){return assembled_;}
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoints(){return boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(points_);}

    int getSize(){return components_.size();}

    double getLikelihood(){return likelihood_;}

    void setDebug(bool debug){debug_=debug;}

    void setUseColor(bool use_color){use_color_=use_color;}



protected:

    //true if the box is assembled based on the components
    bool assembled_;

    bool modelSet_;

    bool debug_;

    bool use_color_; //enable usage of color point clouds. Default is no color;

    //the point cloud representing the complete compoundObject
    pcl::PointCloud<pcl::PointXYZRGB> points_;

    //pose of the object, defined by the middle of the box.
    //Orientation is given by x defining the direction of the switch. -x gives the direction of the battery slot
    base::Pose pose_;

    //Dimensions (minimal bounding box) of the compound object
    base::Vector3d dimensions_;

    //The components of the compound object
    std::vector<TComponent> components_;

    double likelihood_;

    //protected methods

    void deleteComponents(){components_.clear();}
    void deletePoints(){points_.clear();}

};

}

#endif // COMPOUNTOBJECT_H
