#ifndef OPENCV_HELPER_H_
#define OPENCV_HELPER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <dirent.h>

//determined by black magic
static const float MAXDISTANCE = 10;
static const double background_factor = 0.67;

static const double ZERO_D = 1e-7;
static const float ZERO_F = (float)ZERO_D;

bool isZero(float f){
    if(f < ZERO_F && f > -ZERO_F) return true;
    return false;
}

bool isZero(double f){
    if(f < ZERO_D && f > -ZERO_D) return true;
    return false;
}

cv::Vec3d xyToHVec(const cv::Vec2d& xy){
    return cv::Vec3d(xy[0],xy[1],1);
}

cv::Vec3d xyToHVec(double x, double y){
    return cv::Vec3d(x,y,1);
}

void findFeatures(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    cv::Ptr<cv::FeatureDetector> detector;
    detector = cv::xfeatures2d::SURF::create(500, 4, 2, false, false);
    detector->detect(img, keypoints);
    detector->compute(img, keypoints, descriptors);
}

void findFeaturesMask(const cv::Mat& img, const cv::Mat& mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){
    cv::Ptr<cv::FeatureDetector> detector;
    detector = cv::xfeatures2d::SURF::create(500, 4, 2, false, false);
    detector->detect(img, keypoints, mask);
    detector->compute(img, keypoints, descriptors);
}

//builds a mask to be used in findFeaturesMask that filters out the artificial black background in virtual views
void buildMask(const cv::Mat& img, cv::Mat& mask){
    //build a LookUpTable for every color value
    cv::Mat lut(1,256,CV_8U,255);
    //don't keep 0.0 (artificial black from virtual views)
    lut.at<uchar>(0)=0;

    //convert 3 channel image to 1
    cv::Mat img_gray;
    cvtColor(img, img_gray, CV_BGR2GRAY);
    //build the mask by applying the LookUpTable on the image
    LUT(img_gray, lut, mask);
}

//pixel location based radius matching for sequence/stereo images, only consider matches with a maximum distance of MAXDISTANCE pixels
void radiusMatchFrames(const std::vector<cv::KeyPoint>& origin_keypoints, const std::vector<cv::KeyPoint>& current_keypoints, 
            const cv::Mat& origin_descriptors, const cv::Mat& current_descriptors, 
            std::vector<cv::DMatch>& matches){

    matches.clear();

    CV_Assert( current_descriptors.type() == origin_descriptors.type() );

    if( current_descriptors.empty() || origin_descriptors.empty() )
    {
        return;
    }

    int kp_count_prev = origin_descriptors.rows;
    int kp_count_curr = current_descriptors.rows;

    for(int i = 0; i < kp_count_prev; i++ )
    {
        //build mat with descriptors in area of the i-th previous descriptor
        std::vector<uint8_t> current_descriptors_inarea;
        std::vector<int> inarea_curr_index;
        int x=origin_keypoints[i].pt.x;
        int y=origin_keypoints[i].pt.y;

        for(int j = 0; j < kp_count_curr; j++) {
            int distanceX, distanceY;
            distanceX=abs(current_keypoints[j].pt.x-x);
            distanceY=abs(current_keypoints[j].pt.y-y);
            if(distanceX+distanceY < MAXDISTANCE){
                const uint8_t* Mj = current_descriptors.ptr<uint8_t>(j);
                for(int k=0;k<32;k++){
                    current_descriptors_inarea.push_back(*(Mj+k));
                }
                inarea_curr_index.push_back(j);
            }
        }
        if(current_descriptors_inarea.size()<32) {
            continue;
        }
        cv::Mat current_descriptors_inarea_mat(current_descriptors_inarea);
        current_descriptors_inarea_mat=current_descriptors_inarea_mat.reshape(0,current_descriptors_inarea_mat.rows/current_descriptors.cols);

        // use batchdistance to match the i-th previous feature against the features in its area from inarea_mat
        cv::Mat dist, nidx;
        batchDistance(origin_descriptors.row(i), current_descriptors_inarea_mat, dist, CV_32S, nidx,
                cv::NORM_HAMMING, 1, cv::noArray(), 0, true);
        // fix indices to global descriptor indices
        matches.push_back(cv::DMatch(i, inarea_curr_index[nidx.at<int>(0,0)], (float)dist.at<uint8_t>(0,0)));
    }    
}

void convertSensorImageToMat(const sensor_msgs::Image::ConstPtr& msg, cv::Mat& img,
            const std::string encoding=sensor_msgs::image_encodings::BGR8){
    cv_bridge::CvImagePtr cv_ptr;
    try {
        //TODO SPEED: toCvShare
        cv_ptr = cv_bridge::toCvCopy(msg, encoding);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    (cv_ptr.get()->image).copyTo(img);
}

void convertMatToSensorImage(const cv::Mat& img, sensor_msgs::Image& msg,
            const std::string encoding=sensor_msgs::image_encodings::BGR8){
    cv_bridge::CvImage cv_image;
    cv_image.image = img;
    cv_image.encoding = encoding;
    cv_image.toImageMsg(msg);
}

sensor_msgs::ImagePtr makeSharedImage(const sensor_msgs::Image& ros_image) {
    return boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image(ros_image));
}

//reads a single image file and converts it to a cv::Mat
void readImage(const std::string& file_location, cv::Mat& image){
    image = cv::imread(file_location,CV_LOAD_IMAGE_COLOR);
}

//reads a single image file and converts it to a sensor_msgs::Image
void readImage(const std::string& file_location, sensor_msgs::Image& ros_image){
    cv::Mat img;
    readImage(file_location, img);
    convertMatToSensorImage(img, ros_image);
}

//shows a sensor_msgs::Image in an opencv window
void showImage(sensor_msgs::ImageConstPtr& msg){
    cv::Mat img;
    convertSensorImageToMat(msg, img);
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", img);

    cv::waitKey(0);    
}

//shows a cv::Mat in an opencv window
void showImage(cv::Mat& img){
    cv::namedWindow( "Display window", cv::WINDOW_NORMAL);
    cv::imshow( "Display window", img);

    cv::waitKey(0);    
}

//reads every image file in a folder and converts it to a vector of sensor_msgs::Images
void readFolder(const std::string& folder_location, std::vector<sensor_msgs::Image>& images){
    DIR *dir;
    struct dirent *ent;

    char tab2[1024];
    strncpy(tab2, folder_location.c_str(), sizeof(tab2));
    dir = opendir(tab2);

    if (dir != NULL) {
        while ((ent = readdir (dir)) != NULL) {
            std::string filename(ent->d_name);
            size_t index = filename.rfind(".");
            std::string file_ending=filename.substr(index+1,filename.npos);
            if(!file_ending.compare("jpg") || !file_ending.compare("jpeg")) {
                sensor_msgs::Image img;
                readImage(folder_location + filename, img);
                images.push_back(img);
            }
        }
        closedir (dir);
    } else {
        ROS_ERROR_STREAM("Could not open directory");
    }
}

#endif