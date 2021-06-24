/************************************************* 
Copyright:Volcano Mecanum Robot 
Author: 锡城筱凯
Date:2021-06-23 
Blog：https://blog.csdn.net/xiaokai1999
Description:麦轮跑cartographer 3d建图时的传感器启动文件
**************************************************/  
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include <rosgraph_msgs/Clock.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>

using namespace std;

#define TIME_STEP 32                        // 时钟
#define ROBOT_NAME "Mecanum"                // 机器人名称
ros::NodeHandle *n;

static int controllerCount;
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          // 时钟通讯客户端
webots_ros::set_int timeStepSrv;            // 时钟服务数据

double GPSvalues[2];                        // GPS数据
double Inertialvalues[4];                   // imu数据

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());

}

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/Mecanum' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

/*******************************************************
* Function name ：broadcastTransform
* Description   ：TF坐标系转换函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void broadcastTransform()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(GPSvalues[0],GPSvalues[1],0));
    tf::Quaternion q(Inertialvalues[0],Inertialvalues[2],Inertialvalues[1],-Inertialvalues[3]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link"));
    transform.setIdentity();
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "Mecanum/Velodyne_VLP_16"));
}

/*******************************************************
* Function name ：InertialUnitCallback
* Description   ：IMU回调函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void InertialUnitCallback(const sensor_msgs::Imu::ConstPtr &value)
{
    
    Inertialvalues[0] = value->orientation.x;
    Inertialvalues[1] = value->orientation.y;
    Inertialvalues[2] = value->orientation.z;
    Inertialvalues[3] = value->orientation.w;
    broadcastTransform();
}
/*******************************************************
* Function name ：GPSCallback
* Description   ：GPS回调函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void GPSCallback(const geometry_msgs::PointStamped::ConstPtr &value)
{
    GPSvalues[0] = value->point.x;
    GPSvalues[1] = value->point.z;
    broadcastTransform();  
}

int main(int argc,char **argv)
{
    std::string controllerName;
    // 创建一个节点
    ros::init(argc, argv, string(ROBOT_NAME)+string("_init"), ros::init_options::AnonymousName);
    n = new ros::NodeHandle;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
    }
    ros::spinOnce();

    timeStepClient = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("/robot/time_step"));
    timeStepSrv.request.value = TIME_STEP;

    // if there is more than one controller available, it let the user choose
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // leave topic once it is not necessary anymore
    nameSub.shutdown();

    // enable lidar
    ros::ServiceClient set_lidar_client;
    webots_ros::set_int lidar_srv;
    webots_ros::set_bool lidar_bool_srv;
    ros::Subscriber sub_lidar_scan;

    set_lidar_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("/Velodyne_VLP_16/enable"));
    lidar_srv.request.value = TIME_STEP;
    if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) {
        ROS_INFO("Lidar enabled.");
        set_lidar_client = n->serviceClient<webots_ros::set_bool>(string(ROBOT_NAME)+string("/Velodyne_VLP_16/enable_point_cloud"));
        lidar_bool_srv.request.value = 1;
        if (set_lidar_client.call(lidar_bool_srv) && lidar_bool_srv.response.success) {
            ROS_INFO("Lidar_point_cloud enabled.");
        } else {
            if (!lidar_bool_srv.response.success)
            ROS_ERROR("Failed to enable lidar_point_cloud.");
            return 1;
        }
    } else {
        if (!lidar_srv.response.success)
        ROS_ERROR("Failed to enable lidar.");
        return 1;
    }
    

    // enable gps
    ros::ServiceClient set_GPS_client;
    webots_ros::set_int GPS_srv;
    ros::Subscriber sub_GPS;
    ros::Subscriber sub_GPS_speed;
    set_GPS_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("/gps/enable"));
    GPS_srv.request.value = TIME_STEP;
    if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) {
        sub_GPS = n->subscribe(string(ROBOT_NAME)+string("/gps/values"), 1, GPSCallback);
        // sub_GPS_speed = n->subscribe("volcano/gps/speed", 1, GPSspeedCallback);
        while (sub_GPS.getNumPublishers() == 0) {
        }
        ROS_INFO("GPS enabled.");
    } else {
        if (!GPS_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable GPS.");
        return 1;
    }

    // enable inertial unit
    ros::ServiceClient set_inertial_unit_client;
    webots_ros::set_int inertial_unit_srv;
    ros::Subscriber sub_inertial_unit;
    set_inertial_unit_client = n->serviceClient<webots_ros::set_int>(string(ROBOT_NAME)+string("/inertial_unit/enable"));
    inertial_unit_srv.request.value = TIME_STEP;
    if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) {
        sub_inertial_unit = n->subscribe(string(ROBOT_NAME)+string("/inertial_unit/quaternion"), 1, InertialUnitCallback);
        while (sub_inertial_unit.getNumPublishers() == 0) {
        }
        ROS_INFO("Inertial unit enabled.");
    } else {
        if (!inertial_unit_srv.response.success)
        ROS_ERROR("Sampling period is not valid.");
        ROS_ERROR("Failed to enable inertial unit.");
        return 1;
    }

    ROS_INFO("You can now start the creation of the map using 'rosrun gmapping slam_gmapping "
            "scan:=/volcano/Sick_LMS_291/laser_scan/layer0 _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30 _delta:=0.2'.");
    ROS_INFO("You can now visualize the sensors output in rqt using 'rqt'.");

    // main loop
    while (ros::ok()) {
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
        ROS_ERROR("Failed to call service time_step for next step.");
        break;
        }
        ros::spinOnce();
    }
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);

    ros::shutdown();
    return 0;

}