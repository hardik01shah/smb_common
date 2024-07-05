#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_topic", 1);

    std::string pcd_file_path;
    std::string map_frame;
    // nh.param<std::string>("pcd_file_path", pcd_file_path, "");
    nh_.getParam("pcd_file_path", pcd_file_path);
    nh_.getParam("map_frame",map_frame);
    if (pcd_file_path.empty())
    {
        ROS_ERROR("No PCD file path specified. Please set the parameter 'pcd_file_path'.");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", pcd_file_path.c_str());
        return -1;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = map_frame; // Adjust the frame ID as needed

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        output.header.stamp = ros::Time::now();
        pcd_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
