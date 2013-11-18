/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#include <object_detection/Component.hpp>


using namespace object_detection;

Component::Component(base::Pose pose):pose_(pose){


}

void Component::setPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points){

    //take average color of component
    double r=0;
    double g=0;
    double b=0;

    Eigen::Vector3i color;
    points_.clear();    

    BOOST_FOREACH(pcl::PointXYZRGB point,*points){
        points_.push_back(point);
        color=point.getRGBVector3i();
        r+=color[0]/255.0f;
        g+=color[1]/255.0f;
        b+=color[2]/255.0f;
    }


    r/=points->size();
    g/=points->size();
    b/=points->size();

    color_=object_detection::Color(r,g,b);

}
/**
 * @brief returns the likelihood of the component matching the color
 * @param color the color to check
 * @return percentage of color match [0:1]
 */
double Component::matchesColor(object_detection::Color color){

    double value=0.0;

    //set up a normal distributions for the given edge length of the box
    boost::math::normal_distribution<double> redND(0,0.20);
    boost::math::normal_distribution<double> greenND(0,0.20);
    boost::math::normal_distribution<double> blueND(0,0.20);


    double redDiff=color.r-color_.r;
    double greenDiff=color.g-color_.g;
    double blueDiff=color.b-color_.b;

    value=pdf(redND,redDiff)*pdf(greenND,greenDiff)*pdf(blueND,blueDiff)/
            (pdf(redND,0)*pdf(greenND,0)*pdf(blueND,0));


    return value;
}

