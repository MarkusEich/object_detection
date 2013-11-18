#include "ros/ros.h"
#include <object_detection/PointCloudDetection.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/**
 *Test for object detection
 */

using namespace object_detection;

class Tester
{

public:

    Tester(ros::NodeHandle nh):nh_(nh){
        detector_=boost::make_shared<PointCloudDetection>();
        pub_=nh_.advertise<sensor_msgs::PointCloud2>("cloud",10);
    }
    ~Tester(){}

    bool run(){

         pcl::PointCloud<pcl::PointXYZ> cloud;
         sensor_msgs::PointCloud2 out_msg;

         if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../cloud_data/object_scans_10.pcd", cloud) == -1) //* load the file
         {
           PCL_ERROR ("Couldn't read test file\n");
           return false;
         }
         std::cout << "Loaded " << cloud.width * cloud.height << " data points from test_pcd.pcd with the following fields: " << std::endl;

         detector_->setPointcloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
         detector_->setDebug(true);
         detector_->applyOutlierFilter(0.10);
         detector_->applyPassthroughFilter(2.0,1.0,1.0);
         detector_->applyVoxelFilter(0.01);

         detector_->removeHorizontalPlanes(0.40,0.10);
         pcl::toROSMsg(*detector_->getPointCloud(),out_msg);


         out_msg.header.stamp=ros::Time::now();
         out_msg.header.frame_id="/base_link";
         pub_.publish(out_msg);
         std::vector<SharedCylinder> cylinders;
         std::vector<SharedBox> boxes;
         detector_->getCylinders(cylinders);
         detector_->getBoxes(boxes);

         std::cout<<cylinders[0]->diameter<<std::endl;
         std::cout<<cylinders[0]->height<<std::endl;

         std::cout<<boxes[0]->dimensions<<std::endl;


         return true;
    }


private:

    boost::shared_ptr<PointCloudDetection> detector_;

    ros::NodeHandle nh_;
    ros::Publisher pub_;

};

int main(int argc, char** argv){

    ros::init(argc,argv,"ros_detection_test");
    ros::NodeHandle nh;
    Tester tester(nh);
    if (tester.run()) std::cout<<"success!"<<std::endl;
    else
        std::cout<<"fail!"<<std::endl;
    
    return 0;
}



