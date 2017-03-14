#include <ros/ros.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/publisher.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudPublisher {
private:
    std::string tf_frame_;
    ros::NodeHandle nh_, private_nh_;
    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;
public:
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl;
    std::string file_name_, cloud_topic_;

    CloudPublisher() : private_nh_("~") {
        private_nh_.param("frame_id", tf_frame_, std::string("/roboy"));
        //get params from commandline or default from launchfile
        private_nh_.getParam("pcd_path", file_name_);
        private_nh_.param<std::string>("cloud_publisher_topic", cloud_topic_, "/cloud_pcd");

        pub_.advertise (nh_, cloud_topic_.c_str(), 1);
        ROS_INFO ("CloudPublisher - Prepared for Publishing data on topic %s with frame_id %s. Waiting for Pointcloud to load...", nh_.resolveName (cloud_topic_).c_str(), tf_frame_.c_str());
    }

    int init() {
        pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl_tmp;

        //get Pointcloud from PCD file
        if (file_name_ == "" || pcl::io::loadPCDFile (file_name_, cloud_pcl_tmp) == -1)
            return (-1);

        //center around origin
        Eigen::Vector4d center;
        pcl::compute3DCentroid(cloud_pcl_tmp, center);
        pcl::demeanPointCloud(cloud_pcl_tmp, center, cloud_pcl_tmp);

        //align to eigenspace using PCA, i.e. align object with coordinate axes
        pcl::PCA<pcl::PointXYZRGB> pcl_pca(false);
        pcl_pca.setInputCloud(cloud_pcl_tmp.makeShared());
        pcl_pca.project(cloud_pcl_tmp, cloud_pcl_tmp);

        //downsample big models
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_pcl_tmp.makeShared());
        sor.setLeafSize(0.004f, 0.004f, 0.004f);
        sor.filter(cloud_pcl);

        //rotate to fit camera coordinates
        pcl::transformPointCloud(cloud_pcl, cloud_pcl, pcl::getTransformation(0,0,0,0,0,0.5*M_PI));

        //conversion for output
        pcl::PCLPointCloud2 cloud_pcl2;
        pcl::toPCLPointCloud2(cloud_pcl, cloud_pcl2);
        pcl_conversions::fromPCL(cloud_pcl2, cloud_ros);
        cloud_ros.header.frame_id = tf_frame_;

        return (0);
    }

    bool spin() {
        int nr_points = cloud_ros.width * cloud_ros.height;
        std::string fields_list = pcl::getFieldsList(cloud_ros);

        //publish every 5 seconds (big cloud with no changes), rate in hz
        ros::Rate loop_rate(0.2);
        while (ros::ok())
        {
            ROS_DEBUG_ONCE ("CloudPublisher - Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str(), nh_.resolveName (cloud_topic_).c_str(), cloud_ros.header.frame_id.c_str());
            cloud_ros.header.stamp = ros::Time::now();

            //if someone listens: publish every 5 seconds
            if (pub_.getNumSubscribers() > 0)
            {
                ROS_DEBUG ("CloudPublisher - Publishing data to %d subscribers.", pub_.getNumSubscribers());
                pub_.publish (cloud_ros);
            }

            loop_rate.sleep();
        }

        return (true);
    }

};

int main(int argc, char** argv) { 
    ros::init(argc, argv, "CloudPublisher");

    CloudPublisher c;

    if (c.init() == -1) {
        ROS_ERROR ("Could not load file. Exiting.");
        return (-1);
    }

    ROS_INFO ("CloudPublisher - Loaded a point cloud with %d points (total size is %zu) and the following channels: %s. Started Publishing",  c.cloud_ros.width * c.cloud_ros.height, c.cloud_ros.data.size(), pcl::getFieldsList(c.cloud_ros).c_str());

    c.spin();

    return (0);
}
