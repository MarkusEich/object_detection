find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rock_library(object_detection
    SOURCES
        BlobDetection.cpp
        PointCloudDetection.cpp        
        CompoundObject.cpp
        SpacebotBase.cpp
        SpacebotBattery.cpp
        SpacebotCylinder.cpp
        Component.cpp
        Plane.cpp
        Hole.cpp    
    HEADERS 
        Eigen.hpp
        Time.hpp
        Pose.hpp
	Configuration.hpp
	PointCloudConfiguration.hpp
        Blob.hpp
	BlobDetection.hpp
        PointCloudDetection.hpp
	Objects.hpp        
        CompoundObject.hpp
        SpacebotBase.hpp
        SpacebotBattery.hpp
        SpacebotCylinder.hpp
        Component.hpp
        Plane.hpp
        Hole.hpp
    DEPS_PKGCONFIG opencv segmentation
    #This is a problem if multiple versions are installed
    #should take a defined version
    #DEPS_CMAKE PCL 
)

target_link_libraries(object_detection ${PCL_LIBRARIES})
