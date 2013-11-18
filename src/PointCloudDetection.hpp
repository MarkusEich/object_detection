/****************************************************************************
    Markus Eich
    DFKI 2013
    markus.eich@dfki.de
****************************************************************************/

#ifndef POINTCLOUDDETECTION_HPP
#define POINTCLOUDDETECTION_HPP


#include <stdlib.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <object_detection/SpacebotBase.hpp>
#include <object_detection/SpacebotBattery.hpp>

#include <vector>
#include <pcl/segmentation/region_growing.h>

#include <object_detection/PointCloudConfiguration.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointRGBT;

typedef boost::shared_ptr<object_detection::Cylinder> SharedCylinder;
typedef boost::shared_ptr<object_detection::Box> SharedBox;

typedef boost::shared_ptr<std::vector<pcl::PointCloud<PointRGBT> > > SharedPointClusters;


namespace object_detection
{

class PointCloudDetection
{
public:
    PointCloudDetection();

    void setPointcloud(pcl::PointCloud<PointT>::Ptr points);
    void setPointcloud(pcl::PointCloud<PointRGBT>::Ptr points);

    /**
      * @brief Returns current point cloud
      * @return shared pointer of the point cloud
      */
    pcl::PointCloud<PointRGBT>::Ptr getPointCloud(){return boost::make_shared<pcl::PointCloud<PointRGBT> >(pointcloud_);}
    /**
      * @brief Returns point clouds of cylinders
      * @return shared pointer of the point cloud
      */
    SharedPointClusters getCylindersPoints(){return boost::make_shared<std::vector<pcl::PointCloud<PointRGBT> > >(cylinder_clusters_);}
    /**
      * @brief Returns point clouds of boxes
      * @return shared pointer of the point cloud
      */
    SharedPointClusters getBoxesPoints(){return boost::make_shared<std::vector<pcl::PointCloud<PointRGBT> > >(box_clusters_);}


    void setBoxModel(const Box &boxModel){
        boxModel_.color=boxModel.color;
        boxModel_.dimensions=boxModel.dimensions;     
        boxModelSet_=true;
        if (debug_) PCL_WARN("Set box model to %f,%f,%f\n",boxModel.dimensions.x(),boxModel.dimensions.y(),boxModel.dimensions.z());
    }

    void setBaseModel(const Box &boxModel){
        base_.setModel(boxModel);
    }

    void setBatteryModel(const Box &boxModel){
        battery_.setModel(boxModel);
    }

    void setCylinderModel(const Cylinder &cylinderModel){
        cylinderModel_.color=cylinderModel.color;
        cylinderModel_.diameter=cylinderModel.diameter;
        cylinderModel_.height=cylinderModel.height;
        cylinderModelSet_=true;        
        if (debug_) PCL_WARN("Set cylinder model to diameter: %f, height: %f\n",cylinderModel_.diameter,cylinderModel_.height);
    }

    void removeHorizontalPlanes(double minExtension, double maxHorizVariance);

    bool getCylinders(std::vector<object_detection::Cylinder>& cylinders);
    bool getBoxes(std::vector<object_detection::Box>& boxes) __attribute__ ((deprecated));

    //get Base Object, if possible
    bool getBase(object_detection::Box& box);
    pcl::PointCloud<PointRGBT>::Ptr getBasePoints(){return base_.getPoints();}

    //get Battery Object, if possible
    bool getBattery(object_detection::Box& base);
    pcl::PointCloud<PointRGBT>::Ptr getBatteryPoints(){return battery_.getPoints();}

    void applyPassthroughFilter(double x, double y, double z);
    void applyVoxelFilter(double voxel_size);
    void applyOutlierFilter(double radius);    

    void setDebug(bool debug){debug_=debug;}
    void setUseColor(bool use_color){battery_.setUseColor(use_color);base_.setUseColor(use_color);use_color_=use_color;}

    void setUseCylinderOrientation(bool use_cylinder_orientation){use_cylinder_orientation_=use_cylinder_orientation;}

    double matchesColor(const object_detection::Color colorA,const object_detection::Color colorB);

    void setConfiguration( const point_cloud::Configuration& config ) { this->config = config; }
    const point_cloud::Configuration& getConfiguration() const { return config; }

private:

    //private members

    pcl::PointCloud<PointRGBT> pointcloud_; //colored pointcloud    
    pcl::PointCloud<pcl::Normal> cloud_normals_; //Point cloud for storing the cloud_normals


    std::vector<pcl::PointCloud<PointRGBT> > cylinder_clusters_; //colored cylinder point cloud
    std::vector<pcl::PointCloud<PointRGBT> > box_clusters_; //colored cylinder point cloud

    object_detection::Cylinder cylinderModel_; //cylinder model for object search
    object_detection::Box boxModel_; //box model for object search

    bool debug_; //enable debug   
    bool use_color_; //use color for detection
    bool use_cylinder_orientation_; //use cylinder orientation for detection
    double ground_level_;// hight of ground level
    int ransac_iterations_;
    double ransac_dist_threshold_;
    double ransac_normal_dist_weight_;
    double segmentation_cluster_dist_;
    int min_cluster_size_;
    int max_cluster_size_;
    double max_cluster_dist_;
    bool cylinderModelSet_;
    bool boxModelSet_;

    double ransac_min_radius_;
    double ransac_max_radius_;

    point_cloud::Configuration config;

private:

    //Compount Object of the Battery up to 6 planes
    SpacebotBattery battery_;
    //Compount Object of the Base Station up to 6 planes and one hole
    SpacebotBase base_;

    //private functions
    //gets the match of a given Cylinder
    double getCylinderScore(object_detection::Cylinder detectedObject);
    //gets the match of a given Box and a detected rectangularness
    double getBoxScore(object_detection::Box detectedObject, const double& rectangularness);
    //computes normals of the pointcloud member variable
    bool computeNormals();
    //get clusters from the current point cloud
    void getClusters(boost::shared_ptr<std::vector<pcl::PointCloud<PointRGBT> > >& clusters);
    //gets the percentage of a point cloud beeing a cylinder
    double getCylinderLikelihood(
            const pcl::PointCloud<PointRGBT>::Ptr input_cloud_ptr, pcl::ModelCoefficients& coefficients);
    //gets the hight of the cylinder, given a point cloud and the coefficients
    double getCylinderHeight(
            const pcl::PointCloud<PointRGBT>::Ptr input_cloud_ptr, PointT& max_pt, PointT& min_pt, const pcl::ModelCoefficients coefficients);

    void extractPlanes(std::vector <pcl::PointIndices>& clusters);

    double isVertical(base::Vector3d axis);



};

}
#endif // POINTCLOUDDETECTION_HPP
