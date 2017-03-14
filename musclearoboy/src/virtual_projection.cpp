#include <ros/ros.h>

#include <pcl/octree/octree.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include "opencv_helper.cpp"

//intrinsic camera calibration matrix of virtual camera (and real asus xtion) + inverse
cv::Matx33d K;
//for raytracing: world coordinates of image plane borders
cv::Vec3d bl, br, tl, tr;
bool pcl_received=false, direct_projection;
int img_rows, img_cols;
int smoothing;
std::string image_output_path, subscriber_topic;
double roll_stepsize, roll_lowerbound, roll_upperbound, pitch_stepsize, pitch_lowerbound, pitch_upperbound;
double start_x, start_y, start_z, start_yaw;

pcl::PointCloud<pcl::PointXYZRGB> cloud;
message_filters::Subscriber<sensor_msgs::PointCloud2> *pcl_sub;

double radToDeg(double rad){
    return rad * 180 / M_PI;
}

double degToRad(double degree){
    return degree / 180 * M_PI;
}

//write an image to our output path, significantly named by angles
void writeToFile(const cv::Mat& img, double roll, double pitch){
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    std::stringstream path;
    path << image_output_path << "roboy_" <<  std::setprecision(6) << std::setfill('0') << std::setw(4) << std::internal << radToDeg(roll) << "_" << std::setw(3) << (int)round(radToDeg(pitch)) << ".png";
    ROS_INFO_STREAM("VirtualProjection - Outputting image to " << path.str());
    //TODO: create folder if it doesn't exist
    try{
        imwrite(path.str(), img, compression_params);
    } catch (std::runtime_error& ex){
        ROS_WARN("Exception converting image to PNG format: %s\n", ex.what());
        return;
    }   
}

//direct approach: project points via camera matrix, only show the nearest ones
//smoothing: default = 0 = no smoothing, 1 = GaussianBlur, 2 = medianBlur, 3 = bilateralFilter
void projectToImage(double x, double y, double d, double ro, double pi, double ya, int smoothing = 0){
    Eigen::Affine3f transform = pcl::getTransformation(x, y, d, ro, pi, ya);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;
    pcl::transformPointCloud(cloud, cloud_tmp, transform);

    unsigned char img_bgr[img_rows][img_cols][3] = {};
    double img_d[img_rows][img_cols] = {};
    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud_tmp.begin();
    for (; it != cloud_tmp.end(); ++it) {
        cv::Vec3d xyz(it->x, it->y, it->z);
        xyz=K*xyz;
        double de=xyz[2];
        if(de<=0) continue;
        double xd=xyz[0]/de;
        double yd=xyz[1]/de;

        //TODO: actual interpolation for all surrounding pixels instead of first come first serve
        for (int xi=floor(xd); xi<=ceil(xd); xi++){
            for (int yi=floor(yd); yi<=ceil(yd); yi++){
                if(xi <0 || xi >= img_cols) continue;
                if(yi <0 || yi >= img_rows) continue;
                if(de<img_d[yi][xi] || img_d[yi][xi]==0.0) {
                    img_bgr[yi][xi][0]=it->b;
                    img_bgr[yi][xi][1]=it->g;
                    img_bgr[yi][xi][2]=it->r;
                    img_d[yi][xi]=de;
                }
            }
        }
    }
    cv::Mat img(img_rows, img_cols, CV_8UC3, img_bgr);

    cv::Mat img_smoothed;
    switch (smoothing) {
        case 1:
            cv::GaussianBlur(img, img_smoothed, cv::Size(7,7), 0, 0);
            break;
        case 2:
            cv::medianBlur(img, img_smoothed, 3);
            break;
        case 3:
            cv::bilateralFilter(img, img_smoothed, 7, 120, 7);
            break;
        default:
            img_smoothed = img;
            break;
    }
    
    writeToFile(img_smoothed, ro, pi);
}

//alternative raytracing approach: one ray per pixel
//smoothing: default = 0 = no smoothing, 1 = GaussianBlur, 2 = medianBlur, 3 = bilateralFilter
void projectToImage2(double x, double y, double d, double ro, double pi, double ya, int smoothing = 0){
   ROS_DEBUG_STREAM("Virtualprojection - Starting projection");

    Eigen::Affine3f transform = pcl::getTransformation(x, y, d, ro, pi, ya);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;
    pcl::PointCloud<pcl::PointXYZ> cloud_oct;
    pcl::transformPointCloud(cloud, cloud_tmp, transform);
    pcl::copyPointCloud(cloud_tmp, cloud_oct);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_search (0.005f);
    octree_search.setInputCloud(cloud_oct.makeShared());
    octree_search.addPointsFromInputCloud();

    double step_x = (br(0) - bl(0))/img_cols;
    double step_y = (tl(1) - bl(1))/img_rows;
    int count=0;
    unsigned char img_bgr[img_rows][img_cols][3] = {};

    for(int w=0; w < img_cols; w++){
        for(int h=0; h < img_rows; h++){
            std::vector<int> pointIdxVec;
            octree_search.getIntersectedVoxelIndices(Eigen::Vector3f(0.0,0.0,0.0), Eigen::Vector3f(bl(0)+w*step_x,bl(1)+h*step_y,1.0), pointIdxVec, 1);
            if(pointIdxVec.empty()){
                count++;
                img_bgr[h][w][0]=0;
                img_bgr[h][w][1]=0;
                img_bgr[h][w][2]=0;
            } else {
                int r=0, g=0, b=0, s=pointIdxVec.size();
                for(int i=0; i<s; i++){
                    //TODO: median?
                    pcl::PointXYZRGB tmp = cloud_tmp.points[pointIdxVec[i]];
                    r+=tmp.r;
                    b+=tmp.b;
                    g+=tmp.g;
                }
                r/=s;
                b/=s;
                g/=s;
                img_bgr[h][w][0]=b;
                img_bgr[h][w][1]=g;
                img_bgr[h][w][2]=r;
            }
        }
    }
    if(count == img_cols*img_rows) ROS_DEBUG_STREAM("Virtualprojection - no content in image");
    ROS_DEBUG_STREAM("Virtualprojection - Generated Image");

    cv::Mat img(img_rows, img_cols, CV_8UC3, img_bgr);

    cv::Mat img_smoothed;
    switch (smoothing) {
        case 1:
            cv::GaussianBlur(img, img_smoothed, cv::Size(7,7), 0, 0);
            break;
        case 2:
            cv::medianBlur(img, img_smoothed, 3);
            break;
        case 3:
            cv::bilateralFilter(img, img_smoothed, 7, 120, 7);
            break;
        default:
            img_smoothed = img;
            break;
    }
    
    writeToFile(img_smoothed, ro, pi);
}

int spin() {
    //check for a pointcloud every second
    ros::Rate loop_rate(1);
    ROS_DEBUG("Virtualprojection - Started Main Loop");
    while (ros::ok()){
        if (pcl_received){
            ROS_DEBUG("Virtualprojection - Unsubscribing PCL Callback");
            //roll = yz plane clockwise = tilting out of the screen
            //pitch = xz plane coutner clockwise
            //yaw = xy plane clockwise
            ROS_INFO("Virtualprojection - Started writing Images");
            double ro=0.0, pi=0.0;
            for(ro=roll_lowerbound; ro < roll_upperbound; ro+=roll_stepsize){
                if(isZero(ro)) ro=0.0;
                for(pi=pitch_lowerbound; pi < pitch_upperbound; pi+=pitch_stepsize){
                    if(isZero(pi)) pi=0.0;
                    if(direct_projection) projectToImage(start_x, start_y, start_z, degToRad(ro), degToRad(pi), start_yaw, smoothing);
                    else projectToImage2(start_x, start_y, start_z, degToRad(ro), degToRad(pi), start_yaw, smoothing);
                }
                ROS_INFO_STREAM("Virtualprojection - Completed " << ro << " degress roll angle");
            }
            ROS_INFO("Virtualprojection - Wrote all Images");
            return 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return -1;
}

//get pointcloud from cloud publisher, save and unsubscribe when received
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_DEBUG("Virtualprojection - PCL callback starting...");
    
    if(pcl_received){
        return;
    }

    pcl::fromROSMsg (*msg, cloud);

    pcl_sub->unsubscribe();
    ROS_INFO("Virtualprojection - Received Pointcloud");
    pcl_received = true;
}

int init(){
    ros::NodeHandle nh("~");
    nh.getParam("image_output_path", image_output_path);
    nh.param<std::string>("subscriber_topic", subscriber_topic, "/cloud_pcd");
    nh.param<int>("smoothing", smoothing, 1);
    nh.param<int>("resolution_width", img_cols, 640);
    nh.param<int>("resolution_height", img_rows, 480);
    nh.param<bool>("direct_projection", direct_projection, false);
    nh.param<double>("roll_stepsize", roll_stepsize, 1.5);
    nh.param<double>("roll_lowerbound", roll_lowerbound, -45);
    nh.param<double>("roll_upperbound", roll_upperbound, 45); 
    nh.param<double>("pitch_stepsize", pitch_stepsize, 6);
    nh.param<double>("pitch_lowerbound", pitch_lowerbound, 0);
    nh.param<double>("pitch_upperbound", pitch_upperbound, 360);
    nh.param<double>("start_x", start_x, 0.0);
    nh.param<double>("start_y", start_y, 0.0);
    nh.param<double>("start_z", start_z, 2.0);
    nh.param<double>("start_yaw", start_yaw, 0.0);

    //TODO: parameterize more: octree resolution, virtual image position, direct/alternative approach

    K = cv::Matx33d(532.7535127902282, 0.0, 313.026835793531, 0.0, 530.5845173944252, 235.3780808509862, 0.0, 0.0, 1.0);
    cv::Matx33d KI;
    cv::invert(K, KI, cv::DECOMP_SVD);
    bl = KI*cv::Vec3d(0,0,1);
    br = KI*cv::Vec3d(img_cols,0,1);
    tl = KI*cv::Vec3d(0,img_rows,1);
    tr = KI*cv::Vec3d(img_cols,img_rows,1);
    bl/=bl(2);
    tl/=tl(2);
    br/=br(2);
    tr/=tr(2);

    pcl_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, subscriber_topic, 1);
    pcl_sub->registerCallback(boost::bind(callback, _1));

    ROS_DEBUG("Virtual Projection - Initialized");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtualprojection");
    
    /*
    * Debug Output
    * uncomment to see ROS_DEBUG messages
    */
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //    ros::console::notifyLoggerLevelsChanged();
    // }
    
    init();

    return spin();
}