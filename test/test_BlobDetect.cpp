#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <object_detection/BlobDetection.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace object_detection;
namespace bfs = boost::filesystem;

BOOST_AUTO_TEST_CASE( detect_blob_in_image )
{
    // load a test image and see 
    // if the right blobs are detected

    std::string img_path("/home/linki/Desktop/test/test_1_2.jpg");
    cv::Mat cv_image = cv::imread( img_path );

    if (cv_image.empty())
        BOOST_FAIL( std::string("Can not load the image " + img_path) );

    // imread reads the image in BGR color format
    // convert to RGB
    cv::cvtColor(cv_image, cv_image, CV_BGR2RGB);

    // TODO update test values
    std::vector<PrimitiveObject> objs;

    Cylinder cyl;
    cyl.color = Color(0.339007, 0.619089, 0.909447);         // blue
    cyl.diameter = 0.08;
    cyl.height = 0.12;
    objs.push_back( PrimitiveObject( cyl ) );

    Box box_2;
    box_2.color = Color(0.907632, 0.480388, 0.167387);      // orange
    objs.push_back( PrimitiveObject( box_2 ) );   

    Box box;
    box.color = Color(0.830962, 0.748882, 0.350257);         // yellow
    box.dimensions = base::Vector3d( 0.1, 0.05, 0.2 );
    objs.push_back( PrimitiveObject( box ) );

    // configuration for BlobDetection
    ConfigBlobDetection config;
    config.min_area = 50;
    config.min_likelihood = 0.55;
    config.scale = 0.5;
    config.mser_delta = 2;
    config.mser_max_area = 8400;
    config.mser_max_variation = 0.15;
    config.mser_min_diversity = 0.5;

    BlobDetection blob;
    blob.setConfiguration( config );
    blob.setObjectList( objs );

    blob.processImage( cv_image );

    // TODO
    // do some automated test here
    // for some criteria
    // e.g. no more than 3 candidates per blob, and the blob with the highest
    // rating should be within 1% of the true position or so

    // convert RGB debug image back to BGR color space
    cv::Mat cv_debug_image = blob.getDebugImage();
    cv::cvtColor(cv_debug_image, cv_debug_image, CV_RGB2BGR);

    cv::imshow( "Debug Image", cv_debug_image );
    cv::waitKey(0);
}

BOOST_AUTO_TEST_CASE( calc_mean_color )
{
    std::cout << "Calc mean color" << std::endl;

    std::vector<std::string> dir_obj_path;
    dir_obj_path.push_back(std::string("/home/linki/Desktop/images/blue/"));
    dir_obj_path.push_back(std::string("/home/linki/Desktop/images/yellow/"));
    dir_obj_path.push_back(std::string("/home/linki/Desktop/images/orange/"));

    std::string file_extension(".jpg");

    for (unsigned int i = 0; i < dir_obj_path.size(); ++i)
    {
        std::cout << "Folder: " << dir_obj_path.at(i) << std::endl;
        boost::filesystem::path dir(dir_obj_path.at(i));

        std::vector<bfs::path> files;
        bfs::directory_iterator end_itr;
        for (bfs::directory_iterator dir_itr(dir); dir_itr != end_itr; ++dir_itr)
        {
            if (bfs::is_regular_file(*dir_itr)
                    && dir_itr->path().extension().string().compare(file_extension) == 0 )
            {
                files.push_back(*dir_itr);
            }
        }        

        if (files.size() < 0)
        {
            BOOST_FAIL( std::string("No files in directory " + dir_obj_path.at(i)) );
            continue;
        }

        cv::Scalar mean_lab(0.,0.,0.,0.);
        for (unsigned int j = 0; j < files.size(); ++j)
        {
            cv::Mat img_brg = cv::imread(files.at(j).string());
            if (img_brg.empty())
                BOOST_FAIL( std::string("Can not load the image " + files.at(j).string()) );

            cv::Mat frame_lab;
            cv::cvtColor(img_brg, frame_lab, cv::COLOR_BGR2Lab);            
            mean_lab = mean_lab + cv::mean(frame_lab);
        }
        double scale = 1. / files.size();
        mean_lab = mean_lab * scale;

        std::cout << "Lab: " << mean_lab[0] << " " << mean_lab[1] << " " << mean_lab[2] << std::endl;

        Color rgb = Color::fromLab8(mean_lab[0], mean_lab[1], mean_lab[2]);

        std::cout << "color RGB: (" << rgb.r << ", " << rgb.g << ", " << rgb.b << ")" << std::endl;
    }
}