#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

ros::NodeHandle *n;
ros::Publisher pc2_pub;

void pointcloudcallback(const sensor_msgs::PointCloud::ConstPtr& msg){

    sensor_msgs::PointCloud2 pc2_msg;
    pc2_msg.header.frame_id = "odom";
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.height = 1;
    pc2_msg.point_step = 3;
    pc2_msg.is_dense = false;
    pc2_msg.fields.resize(3);
    pc2_msg.fields[0].name = "x";
    pc2_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    pc2_msg.fields[0].count = 1;
    pc2_msg.fields[0].offset = 0;
    pc2_msg.fields[1].name = "y";
    pc2_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    pc2_msg.fields[1].count = 1;
    pc2_msg.fields[1].offset = 4;
    pc2_msg.fields[2].name = "z";
    pc2_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    pc2_msg.fields[2].count = 1;
    pc2_msg.fields[2].offset = 8;
    pc2_msg.is_bigendian = false;
    auto size = msg->points.size();
    pc2_msg.width = size;
    pc2_msg.row_step = size * pc2_msg.point_step;
    //memccpy(pc2_msg.data.data(), msg->points.data(), 1);
    for (int i = 0; i < size; i++)
    {
        pc2_msg.data.push_back(uint8_t(msg->points[i].x));
        pc2_msg.data.push_back(uint8_t(msg->points[i].y));
        pc2_msg.data.push_back(uint8_t(msg->points[i].z));
    }
    std::cout<<pc2_msg.data.size()<<std::endl;
    pc2_pub.publish(pc2_msg);
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "velodyne_pointcloud2", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    pc2_pub = n->advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    
    ros::Subscriber pc_sub = n->subscribe("/Mecanum/Velodyne_VLP_16/point_cloud", 100, pointcloudcallback);
    ros::spin();
    return 0;
    
}