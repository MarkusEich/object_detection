#include "BlobDetection.hpp"
#include <boost/foreach.hpp>

using namespace object_detection;

void BlobDetection::setConfiguration( const ConfigBlobDetection& config )
{
    this->config = config;
}

void BlobDetection::setMaskImage( const std::string& img_path )
{
    cv::Mat raw_mask = cv::imread(img_path);
    cv::resize(raw_mask, mask, cv::Size(), config.scale, config.scale);
}

void BlobDetection::setObjectList( const std::vector<PrimitiveObject> &list )
{
    objList.clear();
    BOOST_FOREACH( const PrimitiveObject &p, list )
    {
        objList.push_back( p.getObject() );
    }
}

/**
 * frame should be in the RGB color space.
 */
void BlobDetection::processImage( const cv::Mat &frame )
{
    candidates.resize( objList.size() );

    dbg_img = frame.clone();

    cv::Mat frame_resized;
    cv::resize(frame, frame_resized, cv::Size(), config.scale, config.scale);

    cv::Mat masked_frame;
    if(!mask.empty())
        frame_resized.copyTo(masked_frame, mask);
    else
        frame_resized.copyTo(masked_frame);

    cv::Mat frame_lab;
    cv::cvtColor(masked_frame, frame_lab, cv::COLOR_RGB2Lab);

    //unsigned int ob = 0;
    for (unsigned int ob = 0; ob < objList.size(); ++ob)
    {

        // convert the required object color from RGB into Lab
        Color target_color;
        uint8_t l = 0;
        uint8_t a = 0;
        uint8_t b = 0;
        objList.at(ob)->color.toLab8(l, a, b);  
        target_color.r = l;
        target_color.g = a;
        target_color.b = b;

        // calculate scalar image
        // each scalar represents color distance metric between color of image pixel and target color
        cv::Mat img_scalar;
        calcEuclideanDistance(frame_lab, img_scalar, target_color);

        // normalize the image and convert to 8 bit
        // 8-bit image is required by MSER function
        cv::Mat img_scalar_8bit;
        img_scalar.convertTo(img_scalar_8bit, CV_8U, 255.0);

        // find blobs
        std::vector<std::vector<cv::Point> > mser_regions;
        try
        {
            float min_area = ( config.min_area * config.scale * config.scale ) / 2.0;
        	cv::MSER(config.mser_delta, 
                    min_area, 
                    config.mser_max_area,
                    config.mser_max_variation, 
                    config.mser_min_diversity, 0, 0., 0., 0)(img_scalar_8bit, mser_regions, cv::Mat());
        } catch (cv::Exception& e )
        {
            throw std::runtime_error( std::string( e.what() ) );
        }

        // calculate the mean color distance inside the blob
        // filter out the blobs according to area
        std::vector<Blob> blob_list;
        boost::math::normal_distribution<double> color_distance_nd(0, 0.1);
        for (unsigned int r = 0; r < mser_regions.size(); ++r)
        {
            float mean_distance = calcMeanInsideBlob( img_scalar, mser_regions.at(r) );
            
            double likelihood = pdf(color_distance_nd, mean_distance) / pdf(color_distance_nd, 0);

            cv::Rect bounding_box = cv::boundingRect( cv::Mat(mser_regions.at(r)) );

            // filter out by area and likelihood
            if (bounding_box.area() > ( config.min_area * config.scale * config.scale )
                   && likelihood > config.min_likelihood)
            {
                cv::Point2f center;
                float radius = 0.0;
                cv::minEnclosingCircle(mser_regions.at(r), center, radius);

                Blob blob;
                blob.x_pos_center = round(center.x / config.scale);
                blob.y_pos_center = round(center.y / config.scale);
                blob.confidence = likelihood;
                blob.diameter = 2. * radius / config.scale;

                blob_list.push_back(blob);
            }
        }

        candidates.at(ob) = blob_list;
    }
}

cv::Mat BlobDetection::getDebugImage()
{
    // take the input image and paint the candidates as blobs 
    for (unsigned int c = 0; c < candidates.size(); ++c)
    {
        Color &target_color = objList.at(c)->color;
        uint8_t red, green, blue = 0;
        target_color.toRGB8(red, green, blue);

        for (unsigned int b = 0; b < candidates.at(c).size(); ++b)
        {
            Blob &blob = candidates.at(c).at(b);
            cv::circle(dbg_img,
                       cv::Point(blob.x_pos_center, blob.y_pos_center),
                       round(blob.diameter / 2.0),
                       cv::Scalar(red, green, blue), 1, 8, 0);

            std::string confidence = boost::lexical_cast<std::string>(blob.confidence);

            float radius = blob.diameter / 2.0;
            int x = blob.x_pos_center;
            int y = blob.y_pos_center - radius - 5.;
            if ( y < 0. )
                y = y + 2. * (radius + 5.);

            cv::putText(dbg_img, confidence, cv::Point(x, y),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 2, cv::Scalar(red, green, blue), 1, CV_AA);
        }
    }

    return dbg_img;
}

std::vector<std::vector<Blob> > BlobDetection::getCandidates() const
{
    assert( candidates.size() == objList.size() ); 
    return candidates;
}

float BlobDetection::calcMeanInsideBlob(const cv::Mat &img_scalar, const std::vector<cv::Point> &blob )
{
    if (img_scalar.depth() != CV_32F)
        throw std::runtime_error( "calcMeanInsideBlob: the depth of img_scalar should be CV_32F" );

    if (img_scalar.channels() != 1)
        throw std::runtime_error( "calcMeanInsideBlob: the number of channels should be 1" );

    unsigned int number = 0;

    // calculate mean of color distance
    float mean_distance = 0.0;
    for (unsigned int p = 0; p < blob.size(); ++p)
    {
        const cv::Point &point = blob.at(p);
        if (number == 0)
            mean_distance = img_scalar.at<float>(point);
        else
            mean_distance = (float)(mean_distance * number) / (number + 1) + img_scalar.at<float>(point) / (number + 1);

        number++;
    }

    return mean_distance;
}

float BlobDetection::calcStdDevInsideBlob(const cv::Mat &img_scalar, const std::vector<cv::Point> &blob, float mean_distance)
{
    if (img_scalar.depth() != CV_32F)
        throw std::runtime_error( "calcStdDevInsideBlob: the depth of img_scalar should be CV_32F" );

    if (img_scalar.channels() != 1)
        throw std::runtime_error( "calcStdDevInsideBlob: the number of channels should be 1" );

    unsigned int number = 0;

    // calculate standard deviation of color distance
    float std_dev_distance = 0.0;
    for (unsigned int p = 0; p < blob.size(); ++p)
    {
        const cv::Point &point = blob.at(p);
        if (number == 0)
            std_dev_distance = std::pow( (img_scalar.at<float>(point) - mean_distance) , 2);
        else
            std_dev_distance = (float)(std_dev_distance * number) / (number + 1)
                    + std::pow( (img_scalar.at<float>(point) - mean_distance) , 2) / (number + 1);

        number++;
    }
    std_dev_distance = std::sqrt(std_dev_distance);

    return std_dev_distance;
}

void BlobDetection::calcEuclideanDistance(const cv::Mat &src, cv::Mat &dst, const Color &target_color)
{
    if (src.depth() != CV_8U)
        throw std::runtime_error( "calcEuclideanDistance: the depth of src image should be CV_8U" );

    if (src.channels() != 3)
        throw std::runtime_error( "calcEuclideanDistance: the number of channels of src image should be 3" );

    dst = cv::Mat::zeros(src.size(), CV_32FC1);
    for (int r = 0; r < src.rows; ++r)
    {
        const cv::Vec3b *src_row = src.ptr<cv::Vec3b>(r);
        float *dst_row = dst.ptr<float>(r);

        for (int c = 0; c < src.cols; ++c)
        {
            // Euclidean
            // take into account just a and b from LAB color model
            //float d1 = ((float)src_row[c][0] - target_color.r) / 255.0;
            float d2 = ((float)src_row[c][1] - target_color.g) / 255.0;
            float d3 = ((float)src_row[c][2] - target_color.b) / 255.0;

            dst_row[c] = std::sqrt(std::pow(d2, 2) + std::pow(d3, 2)); 

        }
    }
}

void BlobDetection::cvtRGB2NRGB(const cv::Mat &src, cv::Mat &dst)
{
    if (src.depth() != CV_8U)
        throw std::runtime_error( "cvtRGB2NRGB: the depth of src image should be CV_8U" );

    if (src.channels() != 3)
        throw std::runtime_error( "cvtRGB2NRGB: the number of channels of src image should be 3" );

    dst = cv::Mat::zeros(src.size(), CV_32FC3);

    for (int r = 0; r < src.rows; ++r)
    {
        const cv::Vec3b *src_row = src.ptr<cv::Vec3b>(r);
        cv::Vec3f *dst_row = dst.ptr<cv::Vec3f>(r);

        for (int c = 0; c < src.cols; ++c)
        {
            float red = src_row[c][0];
            float green = src_row[c][1];
            float blue = src_row[c][2];

            float sum = red + blue + green;

            if (sum != 0.)
            {
                dst_row[c][0] = red / sum;
                dst_row[c][1] = green / sum;
                dst_row[c][2] = blue / sum;
            } else
                dst_row[c][0] = dst_row[c][1] = dst_row[c][2] = 0.0;
        }
    }
}

void BlobDetection::calcAchromaticMask(const cv::Mat src, cv::Mat &dst, float thresh)
{
    if (src.depth() != CV_32F)
        throw std::runtime_error( "calcAchromaticMask: the depth of src image should be CV_32F" );

    if (src.channels() != 3)
        throw std::runtime_error( "calcAchromaticMask: the number of channels of src image should be 3" );

    dst = cv::Mat::zeros(src.size(), CV_8UC1);
    for (int r = 0; r < src.rows; ++r)
    {
        const cv::Vec3f *src_row = src.ptr<cv::Vec3f>(r);
        uchar *dst_row = dst.ptr<uchar>(r);

        for (int c = 0; c < src.cols; ++c)
        {
            // add achromatic filter
            // the pixel which color is close to achromatic (white, grey, black)
            if (std::abs(src_row[c][0] - src_row[c][1]) < thresh &&
                std::abs(src_row[c][0] - src_row[c][2]) < thresh)
            {
                dst_row[c] = 0;
            }
            else
                dst_row[c] = 255.0;
        }
    }
}

void BlobDetection::grayWorldNormalization(const cv::Mat &src, cv::Mat &dst)
{
    if (src.channels() != 3)
        throw std::runtime_error( "grayWorldNormalization: the number of channels of src image should be 3" );

    // calculate mean value for every channel
    // Scalar contains 4 values, first 3 values corresponds 
    // to the mean value of one of the channels
    cv::Scalar mean_values = cv::mean(src);
    float mean_c1 = mean_values[0];
    float mean_c2 = mean_values[1];
    float mean_c3 = mean_values[2];

    // scale the mean value 
    float scaling_norm = cv::sqrt( std::pow(mean_c1, 2) + std::pow(mean_c2, 2) + std::pow(mean_c3, 2) );

    if (scaling_norm == 0.) 
    {
        src.copyTo(dst);
        return;
    }

    mean_c1 = mean_c1 / scaling_norm;
    mean_c2 = mean_c2 / scaling_norm;
    mean_c3 = mean_c3 / scaling_norm;

    // find max scaled mean
    float max_scaled_mean = std::max(mean_c1, mean_c2);
    max_scaled_mean = std::max(max_scaled_mean, mean_c3);

    mean_values[0] = max_scaled_mean / mean_c1;
    mean_values[1] = max_scaled_mean / mean_c2;
    mean_values[2] = max_scaled_mean / mean_c3;

    dst = cv::Mat::zeros(src.size(), CV_8UC3);

    // rows
    for (int r = 0; r < src.rows; ++r)
    {
        const cv::Vec3b *src_ptr = src.ptr<cv::Vec3b>(r);
        cv::Vec3b *dst_ptr = dst.ptr<cv::Vec3b>(r);

        // cols
        for (int c = 0; c < src.cols; ++c)
        {
            // channels
            for (int ch = 0; ch < src.channels(); ++ch)
            {
                float v = src_ptr[c][ch];

                v = v * mean_values[ch];

                if (v > 255.)
                    v = 255.;

                dst_ptr[c][ch] = v; 
            }
        }
    }
}
