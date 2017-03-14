#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include "opencv_helper.cpp"

ros::Publisher pub;
ros::Subscriber sub;
std::string local_folder, subscriber_topic, publisher_topic;
int device_resolution_width, device_resolution_height;

//simulate stereo image pair from single camera
int createStereoImage(const cv::Mat& img_in, cv::Mat& img_out){
    ROS_DEBUG("SmartphoneHMD - Creating stereo image ...");
    cv::Mat img;
    int image_height = img_in.rows;
    int target_height = device_resolution_height;
    if(target_height > image_height) {
        cv::resize(img_in, img, cv::Size((double)target_height / image_height * img_in.cols, target_height), 0, 0, cv::INTER_CUBIC);
    } else {
        cv::resize(img_in, img, cv::Size((double)target_height / image_height * img_in.cols, target_height), 0, 0, cv::INTER_AREA);
    }
    ROS_DEBUG_STREAM("SmartphoneHMD - Stereo image creation - resized image to " << img.cols << " x " << img.rows);

    int image_width = img.cols;
    int overlap=0.1*image_width;
    
    int target_width = 0.5 * device_resolution_width;
    if(target_width > image_width - overlap) target_width = image_width - overlap;



    cv::Mat left_eye, right_eye;
    //use right-aligned image because camera is on the left
    left_eye=img.colRange(image_width - target_width - overlap, image_width - overlap);
    right_eye=img.colRange(image_width - target_width, image_width);

    cv::hconcat(left_eye, right_eye, img_out);

    //write a single jpg and then call
    //mjpg_streamer -i "input_file.so -f /LOCAL_FOLDER -d 0" -o "output_http.so -w /usr/local/www"
    //video can then be found at http://127.0.0.1:8080/?action=stream
    cv::imwrite(local_folder + "frame.jpg", img_out);
    ROS_INFO_STREAM_ONCE("Wrote image to " << local_folder << "frame.jpg");

    return 0;
}

void callback(const sensor_msgs::Image::ConstPtr& msg){
    ROS_DEBUG("SmartphoneHMD - Image Callback - starting ...");
    cv::Mat img_in, img_out;
    convertSensorImageToMat(msg, img_in);

    createStereoImage(img_in, img_out);

    sensor_msgs::Image msg_out;
    convertMatToSensorImage(img_out, msg_out);
    msg_out.header = msg->header;
    pub.publish(msg_out);
    
    ROS_DEBUG("SmartphoneHMD - Image Callback - done");
}

void init(){
    //get params from commandline, launchfile or default values
    ROS_DEBUG_STREAM("SmartphoneHMD - Initializing");
    ros::NodeHandle private_nh("~");
    private_nh.getParam("local_folder", local_folder);
    private_nh.param<std::string>("subscriber_topic", subscriber_topic, "/hmd/input_rgb");
    private_nh.param<std::string>("publisher_topic", publisher_topic, "/hmd/output_rgb");
    private_nh.param<int>("device_resolution_width", device_resolution_width, 1920);
    private_nh.param<int>("device_resolution_height", device_resolution_height, 1080);

    sub = private_nh.subscribe(subscriber_topic, 1, callback);
    pub = private_nh.advertise<sensor_msgs::Image>(publisher_topic.c_str(), 1);
    ROS_DEBUG_STREAM("SmartphoneHMD - Initialized");
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "SmartphoneHMD");
    
    /*
    * Debug Output
    * uncomment to see ROS_DEBUG messages
    */
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //    ros::console::notifyLoggerLevelsChanged();
    // }

    init();

    ros::spin();

    return 0;
}