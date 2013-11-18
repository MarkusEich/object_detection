#include <object_detection/PointCloudDetection.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/program_options.hpp>

namespace po=boost::program_options;
using namespace object_detection;

int main(int argc, char** argv)
{
    // load a point cloud and check for objects

    po::variables_map vm;

    try{
       po::options_description desc("Allowed options");
       desc.add_options()
         ("help", "produce help message")
         ("filename", po::value<std::string>(), "PC filename")
         ;


       po::store(po::parse_command_line(argc, argv, desc), vm);
       po::notify(vm);

       if (vm.count("help")) {
         std::cout << desc << "\n";
         return 1;
       }

       if (vm.count("filename")) {
         std::cout << "Reading file "<<
       vm["filename"].as<std::string>()<<".\n";
       }
       else{
         std::cout << "Error. Please specify --filename option.\n";
         return 1;
       }
    }
    catch (std::exception& e){
       std::cerr << "error: " <<e.what()<<"\n";
       return 1;
     }

    
    pcl::PointCloud<PointRGBT> cloud;
    PointCloudDetection detector;
    point_cloud::Configuration config;
    Cylinder cylinder_model;
    Box box_model;
    pcl::visualization::PCLVisualizer viewer ("Object viewer"); //visualizer

    std::vector<Cylinder> cylinders;   

    Box spacebot_base;
    Box spacebot_battery;

    std::string filename(vm["filename"].as<std::string>());

    std::string type(filename,filename.length()-3,3);


    int success;

    if (type.compare("pcd")==0){
        PCL_INFO("Loading pcd file\n");
        success=pcl::io::loadPCDFile<PointRGBT> (filename, cloud);
    } else
    if (type.compare("ply")==0){
        PCL_INFO("Loading ply file\n");       
        success=pcl::io::loadPLYFile<PointRGBT>(filename,cloud);
    }
    else{
        PCL_ERROR("Unknown file type\n");
        success=-1;
    }

    PointRGBT white(255,255,255);
    //check if there are colors. If not use white as default color
    BOOST_FOREACH(PointRGBT& point,cloud){
        if (point.rgb==0){
            point.rgb=white.rgb;
        }
    }


    if (success>=0)
    {

        time_t start,end;
        time (&start);

        PCL_INFO("Loaded %d data points\n",cloud.width * cloud.height);

        viewer.setBackgroundColor(0, 0, 0);
        viewer.addCoordinateSystem (1.0);
        viewer.initCameraParameters ();

        detector.setDebug(true);
        detector.setPointcloud(boost::make_shared<pcl::PointCloud<PointRGBT> >(cloud));
        detector.setUseColor(false);
        detector.setUseCylinderOrientation(true);

        //filter point cloud

        detector.applyPassthroughFilter(2.0,1.0,1.0);
        detector.applyOutlierFilter(0.10);
        detector.applyVoxelFilter(0.01);      

        //set params
        config.setPlaneParams(1.0,6.0);
        config.setBoxModelThreshold(0.7);
        config.setCylinderModelThreshold(0.7);
        config.setExpectedGroundDistance(0.00);
        detector.setConfiguration(config);

        viewer.addPointCloud(detector.getPointCloud(),"filter");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filter");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0,1.0,0.0, "filter");


        //remove all planes which are larger 50cm and 0.1 radian to a horizontal plane        
        detector.removeHorizontalPlanes(0.8,0.10);

        viewer.addPointCloud(detector.getPointCloud(),"no_ground");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "no_ground");        


        ////////////////////////////////////////////////////////////////////////////////////////////////
        /* Set and extract cylinder
         * Get the features
         * Get the points and visualize them
         */

        //Spacebot Cylinder Model

        cylinder_model.diameter=0.08;
        cylinder_model.height=0.12;
        cylinder_model.color=Color(0.0,0.0,1.0);
        detector.setCylinderModel(cylinder_model);


         if (detector.getCylinders(cylinders)){
             std::vector<Cylinder>::iterator iterC=cylinders.begin();

             pcl::ModelCoefficients coeff;

             coeff.values.resize(7);
             coeff.values[0]=iterC->position.x();
             coeff.values[1]=iterC->position.y();
             coeff.values[2]=iterC->position.z();
             coeff.values[3]=iterC->axis.x();
             coeff.values[4]=iterC->axis.y();
             coeff.values[5]=iterC->axis.z();
             coeff.values[6]=cylinder_model.diameter/2.0f;

             viewer.addCylinder(coeff,"cylinder_module");
             viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "cylinder_module");

             for (;iterC!=cylinders.end();iterC++){
                 std::cout<<"Cylinder Diameter: "<<iterC->diameter<<std::endl;
                 std::cout<<"Cylinder Height: "<<iterC->height<<std::endl;
                 std::cout<<"Cylinder Likelihood: "<<iterC->likelihood<<std::endl;
                 std::cout<<"Cylinder Position: "<<endl;
                 std::cout<<iterC->position<<std::endl;
                 std::cout<<"Cylinder Orientation: (axis): "<<iterC->axis[0]<<","<<iterC->axis[1]<<","<<iterC->axis[2]<<std::endl;                 
             }
         }

         //get points from cylinders and display
         int id=0;
         SharedPointClusters cylinderClusters;
         cylinderClusters=detector.getCylindersPoints();

         std::vector<pcl::PointCloud<PointRGBT> >::iterator clusterIterator=cylinderClusters->begin();

         for(;clusterIterator!=cylinderClusters->end();clusterIterator++){
          std::stringstream ss;
          ss << "cylinder";
          ss << id;
          viewer.addPointCloud(boost::make_shared<pcl::PointCloud<PointRGBT> >(*clusterIterator),ss.str());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str());
          id++;
         }


        ////////////////////////////////////////////////////////////////////////////////////////////////
        /* Set and extract Base using the new component based approach
         * Get the features
         * Get the points and visualize them
         */
        box_model.dimensions=Eigen::Vector3d(0.40,0.20,0.30);
        box_model.color=Color(0.7,0.1,0.1);
        detector.setBaseModel(box_model);

        if (detector.getBase(spacebot_base)){
            std::cout<<"Base Dimension: "<<std::endl;
            std::cout<<spacebot_base.dimensions<<std::endl;
            std::cout<<"Base Likelihood: "<<spacebot_base.likelihood<<std::endl;
            std::cout<<"Base Position: "<<std::endl;
            std::cout<<spacebot_base.position<<std::endl;

            /*visualize all points of the compount object*/
            pcl::PointCloud<PointRGBT>::Ptr basePC=detector.getBasePoints();

            viewer.addPointCloud(basePC,"base");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "base");
            //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0.0,0.0, "base");

            Eigen::Vector3f pos=spacebot_base.position.cast<float>();
            Eigen::Quaternionf orient=spacebot_base.orientation.cast<float>();
            viewer.addCube(pos,orient,box_model.dimensions.x(),box_model.dimensions.y(),box_model.dimensions.z(),"base_cube");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "base_cube");

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////
        /* Set and extract Battery using the new component based approach
         * Get the features
         * Get the points and visualize them
         */

        box_model.dimensions=Eigen::Vector3d(0.20,0.04,0.10);
        box_model.color=Color(0.6,0.6,0.1);
        detector.setBatteryModel(box_model);

        if (detector.getBattery(spacebot_battery)){
            std::cout<<"Battery Dimension: "<<std::endl;
            std::cout<<spacebot_battery.dimensions<<std::endl;
            std::cout<<"Battery Likelihood: "<<spacebot_battery.likelihood<<std::endl;
            std::cout<<"Battery Position: "<<std::endl;
            std::cout<<spacebot_battery.position<<std::endl;

            /*visualize all points of the compount object*/
            pcl::PointCloud<PointRGBT>::Ptr batteryPC=detector.getBatteryPoints();

            viewer.addPointCloud(batteryPC,"battery");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "battery");
            //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,1.0,0.0, "battery");

            Eigen::Vector3f pos=spacebot_battery.position.cast<float>();
            Eigen::Quaternionf orient=spacebot_battery.orientation.cast<float>();
            viewer.addCube(pos,orient,box_model.dimensions.x(),box_model.dimensions.y(),box_model.dimensions.z(),"battery_cube");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "battery_cube");
        }



        time (&end);
        double dif = difftime (end,start);
        PCL_ERROR ("Elapsed time is %.2lf seconds.\n", dif );

        while(!viewer.wasStopped())
        {
            viewer.spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

    }

    else{
         PCL_ERROR ("Couldn't read file %s\n",vm["filename"].as<std::string>().c_str());
    }

}
