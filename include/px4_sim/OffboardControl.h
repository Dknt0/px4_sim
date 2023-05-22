/**
 * PX4 quadrotor control class
 * 
 * Dknt 2023.5
*/

#ifndef OFFBOARDCONTROL_H
#define OFFBOARDCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Image.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <future>

#include <unistd.h>
#include <termios.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define VELOCITY_FORWARD 1.0f
#define VELOCITY_LEFT 1.0f
#define VELOCITY_UP 1.0f
#define VELOCITY_YAW 1.0f

using std::cout;
using std::cin;
using std::endl;
using std::flush;

class OffboardControl {
public:
    OffboardControl(ros::NodeHandle &nh,
                    std::string &prefix, 
                    int rate=20,
                    bool using_cv=true,
                    std::string _camera_topic="/stereo_camera/left/image_raw");

    void init(ros::NodeHandle &nh);
    void start();
    void offboard();
    void arm();
    void disarm();
    void takeoff(double altitude);
    void land();
    void keyboard_vel_ctr();
    void goto_point();

private:
    /* Static members func */
    static void p_hover(void *_this, void *_fut);
    static void p_keyboard_vel_get(void *_this, void *_pro);
    static void p_img_display(void *_this);

    /* Members */
    std::string _prefix; // 前缀
    ros::Rate _rate = ros::Rate(20);
    // cv
    bool _using_cv = true; // 是否使用 OpenCV GUI
    std::string _camera_topic = "/stereo_camera/left/image_raw"; // 相机话题名
    char _key_cv = -1; // OpenCV 窗口返回键值

    // Topic cache
    mavros_msgs::State _drone_state = mavros_msgs::State(); // 无人机状态
    geometry_msgs::PoseStamped _loc_pos_pose = geometry_msgs::PoseStamped(); // 无人机位姿
    geometry_msgs::TwistStamped _loc_pos_vel = geometry_msgs::TwistStamped(); // 无人机速度

    /* Members with mutex */
    // geometry_msgs::Twist keyboard_vel_setpoint;
    mavros_msgs::PositionTarget keyboard_vel_setpoint;
    std::mutex m_keyb_vel_setpoint;
    sensor_msgs::Image _img = sensor_msgs::Image(); // 相机消息
    std::mutex m_img;

    /* ROS communication */
    // pubs
    ros::Publisher _setpoint_pos_pub; // 位置控制 目标点话题
    ros::Publisher _setpoint_row_pub; // 速度控制 目标点话题
    // subs
    ros::Subscriber _state_sub; // 监听无人机状态
    ros::Subscriber _loc_pos_pose_sub; // 监听位置
    ros::Subscriber _loc_pos_vel_sub; // 监听速度
    ros::Subscriber _img_sub; // 监听速度
    // clients
    ros::ServiceClient _arming_client; // 解锁服务 客户端
    ros::ServiceClient _set_mode_client; // 模式更改服务 客户端
    ros::ServiceClient _land_client; // 着陆服务 客户端

};

char getch();

#endif // !OFFBOARDCONTROL_H
