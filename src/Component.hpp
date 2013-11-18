/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef COMPONENT_H
#define COMPONENT_H


#include <base/Eigen.hpp>
#include <base/Pose.hpp>

#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
#include <vector>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <object_detection/Objects.hpp>
#include <boost/math/distributions/normal.hpp>

typedef boost::shared_ptr<std::vector<base::Vector3d> > Polygon_ptr;

namespace object_detection{

class Component
{



public:
    Component(base::Pose pose);

    //Copy Constructor
    Component(const Component& component)
    {
        //copy poses;
        pose_=component.pose_;
        color_=component.color_;
        //copy all points;
        BOOST_FOREACH(pcl::PointXYZRGB point,component.points_){
            points_.push_back(point);
        }
        //copy polygon;
        BOOST_FOREACH(base::Vector3d point,component.polygon_){
            polygon_.push_back(point);
        }

    }

    inline void setHull(const std::vector<base::Vector3d>& polygon){polygon_=polygon;}
    inline Polygon_ptr getHull(){return boost::make_shared<std::vector<base::Vector3d> > (polygon_);}

    void setPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoints(){return points_.makeShared();}

    inline base::Pose getPose(){return pose_;}
    inline base::Vector3d getPosition(){return pose_.position;}
    inline base::Quaterniond getQuaternion(){return pose_.orientation;}
    inline object_detection::Color getColor(){return color_;}
    double matchesColor(object_detection::Color color);




protected:

    object_detection::Color color_;

    base::Pose pose_;
    //The bounding polygon
    std::vector<base::Vector3d> polygon_;
    pcl::PointCloud<pcl::PointXYZRGB> points_;


};

}

#endif // COMPONENT_H
