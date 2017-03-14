#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include "opencv_helper.cpp"

class ImagePublisher {
private:
    ros::NodeHandle private_nh;
    cv::VideoCapture cap;
    cv::Mat image;
    bool live;
    ros::Publisher pub;
    std::string local_folder, publisher_topic, stream_url;

public:
    ImagePublisher() : private_nh("~") {
        //get params from commandline, launchfile or default values
        private_nh.getParam("local_folder", local_folder);
        private_nh.param<std::string>("publisher_topic", publisher_topic, "/hmd/input_rgb");
        private_nh.param<std::string>("stream_url", stream_url, "http://192.168.0.11:8080/video?x.mjpeg");
        private_nh.param<bool>("live", live, true);

        pub = private_nh.advertise<sensor_msgs::Image>(publisher_topic.c_str(), 1);
    }

    int init() {
        if(live) {
            ROS_INFO("ImagePublisher - Prepared for receiving live image data from stream url %s and publishing on topic %s.", stream_url.c_str(), publisher_topic.c_str());

            cap.open(stream_url);

            if (!cap.isOpened())
            {
                ROS_ERROR("ImagePublisher - Cannot open the video stream");
                return -1;
            }

            ROS_INFO("ImagePublisher - Opened video stream");

            ROS_INFO_STREAM("ImagePublisher - Frame size : " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << " x " << cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        } else {
            ROS_INFO ("ImagePublisher - Prepared for receiving offline image data from file in %s and publishing on topic %s.", local_folder.c_str(), publisher_topic.c_str());
            image=cv::imread(local_folder+"sample_frame.jpg", CV_LOAD_IMAGE_COLOR);

            if(!image.data ) {
                ROS_ERROR("ImagePublisher - Could not open or find the image");
                return -1;
            }

            ROS_INFO_STREAM("ImagePublisher - Frame size : " << image.cols << " x " << image.rows);
        }
    }

    //main loop: update file if live, publish
    int spin() {
        bool sample_written = false;

        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            if(live){
                bool bSuccess = cap.read(image);
                if (!bSuccess) {
                    ROS_ERROR("Cannot read a frame from input video stream. Please restart when the stream is online again.");
                    return -1;
                }
                if(!sample_written) {
                    cv::imwrite(local_folder+"sample_frame.jpg", image);
                    ROS_INFO_STREAM("Wrote sample image to '" << local_folder << "sample_frame.jpg'");
                    sample_written = true;
                }
            }

            sensor_msgs::Image ros_img;
            convertMatToSensorImage(image, ros_img);
            ros_img.header.stamp = ros::Time::now();

            if (pub.getNumSubscribers() > 0)
            {
                ROS_DEBUG("Publishing data to %d subscribers.", pub.getNumSubscribers());
                pub.publish(ros_img);
            }

            loop_rate.sleep();
        }

        return 0;
    }
};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ImagePublisher");

    /*
    * Debug Output
    * uncomment to see ROS_DEBUG messages
    */
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //    ros::console::notifyLoggerLevelsChanged();
    // }
    
    ImagePublisher ip;

    if (ip.init() == -1) {
        ROS_ERROR ("Could not load file. Exiting.");
        return (-1);
    }

    return ip.spin();
}