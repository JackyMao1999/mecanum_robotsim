/************************************************* 
Copyright:Volcano Mecanum Robot 
Author: 锡城筱凯
Date:2021-06-23 
Blog：https://blog.csdn.net/xiaokai1999
Description:麦轮小车的速度控制文件
**************************************************/
#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
 
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
 
using namespace std;

#define TIME_STEP 32                        //时钟
#define NMOTORS 4                           //电机数量
#define MAX_SPEED 2.0                       //电机最大速度
 
ros::NodeHandle *n;

static int controllerCount;                 // 控制器数量
static std::vector<std::string> controllerList; 

ros::ServiceClient timeStepClient;          // 时钟通讯客户端
webots_ros::set_int timeStepSrv;            // 时钟服务数据

ros::ServiceClient set_velocity_client;     // 速度服务客户端
webots_ros::set_float set_velocity_srv;     // 速度服务数据

ros::ServiceClient set_position_client;     // 电机位置服务客户端
webots_ros::set_float set_position_srv;     // 电机位置服务数据

double speeds[NMOTORS];
//匹配电机名
static const char *motorNames[NMOTORS] ={"motor_left_front", "motor_left_back", "motor_right_front","motor_right_back"};

/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed() {   
    
    for (int i = 0; i < NMOTORS; ++i) {
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/Mecanum/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = speeds[i];
        set_velocity_client.call(set_velocity_srv);
    }
}

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
* Function name ：init_controller
* Description   ：初始化所有控制器
* Parameter     ：无
* Return        ：无
**********************************************************/
int init_controller(){
    string controllerName;
    // 订阅model_name话题获取控制器
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    } 
    ros::spinOnce();

    // 如果有多个控制器存在，让用户选择
    if (controllerCount == 1)   
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want touse:\n";   
        std::cin >> wantedController;   
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];   
        else {
            ROS_ERROR("Invalid number for controller choice.");  
            return 1;
        }
    } 
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // 关闭topic
    nameSub.shutdown();
}

/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the '/volcano' node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

/*******************************************************
* Function name ：init_motors
* Description   ：初始化电机函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void init_motors(){
    // init motors 
    for (int i = 0; i < NMOTORS; ++i) {
        // position速度控制时设置为缺省值INFINITY   
        
        set_position_client = n->serviceClient<webots_ros::set_float>(string("/Mecanum/") + string(motorNames[i]) + string("/set_position"));   
        set_position_srv.request.value = INFINITY;
        if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
            ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]); 
        // speed，发送电机速度给wheel1-6   
        // ros::ServiceClient set_velocity_client;
        // webots_ros::set_float set_velocity_srv;   
        set_velocity_client = n->serviceClient<webots_ros::set_float>(string("/Mecanum/") + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 0.0;   
        if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Velocity set to 0.0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }   

}

/*******************************************************
* Function name ：keyboardDataCallback
* Description   ：webots按键数据回调函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void keyboardDataCallback(const webots_ros::Int32Stamped::ConstPtr &value){
    int send =0;
    float lspeed=0,rspeed=0;
    ROS_INFO("sub keyboard value = %d",value->data);
    switch (value->data)
    {
        // <
        case 314:
            speeds[0] = -3.0;
            speeds[1] = 3.0;
            speeds[2] = 3.0;
            speeds[3] = -3.0;
            send=1;
            break;
        // ^
        case 315:
            speeds[0] = 2.0;
            speeds[1] = 2.0;
            speeds[2] = 2.0;
            speeds[3] = 2.0;
            send=1;
            break;
        // >
        case 316:
            speeds[0] = 3.0;
            speeds[1] = -3.0;
            speeds[2] = -3.0;
            speeds[3] = 3.0;
            send=1;
            break;
        // 👇
        case 317:
            speeds[0] = -2.0;
            speeds[1] = -2.0;
            speeds[2] = -2.0;
            speeds[3] = -2.0;
            send=1;
            break;
        // space
        case 32:
            speeds[0] = 0;
            speeds[1] = 0;
            speeds[2] = 0;
            speeds[3] = 0;
            send=1;
            break;
        default:
            send=0; 
            break;
    }
    if (send)
    {
        updateSpeed();
        send=0;
    } 
}

/*******************************************************
* Function name ：init_keyboard_enable
* Description   ：初始化键盘使能函数
* Parameter     ：无
* Return        ：无
**********************************************************/
void init_keyboard_enable(){
    ros::ServiceClient keyboardEnableClient;
    webots_ros::set_int keyboardEnablesrv;
   
    keyboardEnableClient = n->serviceClient<webots_ros::set_int>("/Mecanum/keyboard/enable");
    keyboardEnablesrv.request.value = TIME_STEP;
    if (keyboardEnableClient.call(keyboardEnablesrv) && keyboardEnablesrv.response.success)
    {
        ros::Subscriber keyboardSub;
        keyboardSub = n->subscribe("/Mecanum/keyboard/key",1,keyboardDataCallback);
        //ros::spinOnce();
        while (keyboardSub.getNumPublishers() == 0) {}
        ROS_INFO("Keyboard enabled.");
        ROS_INFO("Use the arrows in Webots window to move the robot.");
        ROS_INFO("Press the End key to stop the node.");
         // main loop 
        while (ros::ok()) {   
            ros::spinOnce();
            if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
            {  
                ROS_ERROR("Failed to call service time_step for next step.");     
                break;   
            }   
            ros::spinOnce();
        } 
        keyboardEnablesrv.request.value = 0;
        keyboardEnableClient.call(keyboardEnablesrv);
    }
    else
    ROS_ERROR("Could not enable keyboard, success = %d.", keyboardEnablesrv.response.success);
}

int main(int argc, char **argv) {
   
    // create a node named '/Mecanum' on ROS network
    ros::init(argc, argv, "Mecanum_robot_velocity", ros::init_options::AnonymousName);

    n = new ros::NodeHandle;  
    signal(SIGINT, quit);   //SIGINT 交互注意信号
    
    if (init_controller()==1) return 1;
    
    // 控制webots时钟
    timeStepClient = n->serviceClient<webots_ros::set_int>("/Mecanum/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    init_motors();
    init_keyboard_enable();

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;

}
