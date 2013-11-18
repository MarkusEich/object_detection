/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#include <object_detection/Plane.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace object_detection;

Plane::Plane(double rectangularness, double area, double width,double height,  base::Pose pose,base::Vector3d normal,base::Vector3d expansion,double aboveGround):Component(pose)
{    
    rectangularness_=rectangularness;
    area_=area;

    width_=width;
    height_=height;

    pose_=pose;    
    normal_=normal;

    expansion_=expansion;

    //Flip the plane normal towards the origin view point
    base::Vector3d vp=-pose.position;

    if (vp.dot (normal_) < 0)
            normal_ *= -1;



}
/** @brief gets the score of how parallel two planes are to each other in an interval [0,1]
 *  @param otherPlane the plane to compare with
 *  @return parallel score
 */
double Plane::isParallel(object_detection::Plane& otherPlane)
{
    double value = fabs(1-fabs((getNormal().dot(otherPlane.getNormal())-(getNormal().norm()*otherPlane.getNormal().norm()))));

    if (value<0.5) //cut off
        return 0.0;
    else
        return value;

}
/** @brief gets the score of how orthogonal two planes are to each other in an interval [0,1]
 *  @param otherPlane the plane to compare with
 *  @return parallel score
 */
double Plane::isOrthogonal(object_detection::Plane& otherPlane)
{    
    double value =fabs(1-fabs(getNormal().dot(otherPlane.getNormal())));
        if (value<0.5) //cut off
           return 0.0;
    else
        return value;

    return value;

}
/** @brief gets the distance between to planes based on the COG
 *  @param otherPlane the plane to compare with
 *  @return parallel distance
 */
double Plane::getCogDistance(object_detection::Plane& otherPlane)
{

    base::Vector3d cog=otherPlane.getPosition();
    double distance=sqrt(pow(cog.x()-getPosition().x(),2)+pow(cog.y()-getPosition().y(),2)+pow(cog.z()-getPosition().z(),2));

    return distance;

}
/** @brief Checks if two extracted planes share a common edge based on the direction of the edge vector, the length and the maximal distance of the vector
 *  @param otherPlane the plane to compare with
 *  @return likelihood
 */
bool Plane::hasCommonEdge(object_detection::Plane& otherPlane)
{

    bool value=false;

    //TODO implement
    return value;

}
/** @brief Checks if a shape matches the given plane based on rectangularness and size
 *  @param rectangularness between 0 and 1
 *  @param width
 *  @param height
 *  @return likelihood
 */
double Plane::matchesShape(double rectangularness,double width,double height){

    double likelihood=0.0;
    double temp;

    //sort width and height
    if (width<height){
        temp=height;
        height=width;
        width=temp;
    }

    //set up a normal distributions for the given edge length of the box
    boost::math::normal_distribution<double> widthND(0,0.05);
    boost::math::normal_distribution<double> heigthND(0,0.05);
    boost::math::normal_distribution<double> rectND(0,0.10);


    double widthDiff=width-width_;
    double heigthDiff=height-height_;
    double rectDiff=1.0-rectangularness;

    likelihood=pdf(widthND,widthDiff)*pdf(heigthND,heigthDiff)*pdf(rectND,rectDiff)/
            (pdf(widthND,0)*pdf(heigthND,0)*pdf(rectND,0));

    return likelihood;


}

/**
 * @brief Method for detecting a hole in a plane. Used to get the slot. Only size of hole matters
 * @param width of the slot (y direction)
 * @param height of the slot (z direction)
 * @return found a matching slot
 */
bool Plane::hasSlot(double width,double height){

    if (points_.size()==0){
        PCL_WARN("Error! No points in plane to detect slot\n");
        return false;
    }

    segmentation::Plane plane;
    std::list<std::list<std::pair<segmentation::Shared_Point, segmentation::Shared_Point> > > shapes;
    boost::shared_ptr<cv::Mat> image;
    double scale;

    plane.points.clear();
    BOOST_FOREACH(pcl::PointXYZRGB point,points_){
        plane.addPoint(segmentation::Shared_Point(new segmentation::Point(point.x,point.y,point.z)));
    }

    plane.setAlpha(0.001);

    shapes=plane.getHull();
    image=plane.getHullCVImagesNofill(scale);

    /*debug block. Gets and shows filled alpha hull
    cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", *image );
    cv::waitKey(0);        
    cv::destroyWindow("Display window");
    end debug block*/

    //dirty hack. but works for the contest
    //TODO make a shape analysis for the slot

    if (shapes.size()>1)
    {
        PCL_WARN("Slot found\n");
        return true;
    }
    else
    {
        PCL_WARN("Slot NOT found\n");
        return false;
    }

}







