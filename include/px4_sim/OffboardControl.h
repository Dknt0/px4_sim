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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
#define VELOCITY_YAW 0.5f

#define INIT_POS_X 0.0
#define INIT_POS_Y -40.0

// UBUNTU/LINUX terminal color codes
#define RESET       "\033[0m"
#define BLACK       "\033[30m"          /* Black */
#define RED         "\033[31m"          /* Red */
#define GREEN       "\033[32m"          /* Green */
#define YELLOW      "\033[33m"          /* Yellow */
#define BLUE        "\033[34m"          /* Blue */
#define MAGENTA     "\033[35m"          /* Magenta */
#define CYAN        "\033[36m"          /* Cyan */
#define WHITE       "\033[37m"          /* White */
#define BOLDBLACK   "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"   /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"   /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"   /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"   /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"   /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"   /* Bold White */

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

    void path_planning(double _x, double _y, double _yaw);

    // commination with move_base
    // void set_move_base_target(double _x, double _y, double _yaw);

    static void p_hover(void *_this, void *_fut);

private:
    /* Static members func */
    static void p_keyboard_vel_get(void *_this, void *_pro);
    static void p_img_display(void *_this);

    /* Members */
    std::string _prefix; // 前缀
    ros::Rate _rate = ros::Rate(20);
    // cv
    bool _using_cv = true; // 是否使用 OpenCV GUI
    std::string _camera_topic = "/stereo_camera/left/image_raw"; // 相机话题名
    char _key_cv = -1; // OpenCV 窗口返回键值
    // move_base
    bool _planning = false;

    // Topic cache
    mavros_msgs::State _drone_state = mavros_msgs::State(); // 无人机状态
    geometry_msgs::PoseStamped _loc_pos_pose = geometry_msgs::PoseStamped(); // 无人机位姿
    geometry_msgs::TwistStamped _loc_pos_vel = geometry_msgs::TwistStamped(); // 无人机速度
    geometry_msgs::Twist _cmd_vel = geometry_msgs::Twist(); // 路径规划指令

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
    ros::Publisher _movebase_target_pub; // 路径规划目标点
    // subs
    ros::Subscriber _state_sub; // 监听无人机状态
    ros::Subscriber _loc_pos_pose_sub; // 监听位置
    ros::Subscriber _loc_pos_vel_sub; // 监听速度
    ros::Subscriber _img_sub; // 监听速度
    ros::Subscriber _cmd_vel_sub; // 监听导航指令
    // clients
    ros::ServiceClient _arming_client; // 解锁服务 客户端
    ros::ServiceClient _set_mode_client; // 模式更改服务 客户端
    ros::ServiceClient _land_client; // 着陆服务 客户端
    // Action clients
    // std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> _move_base_client;

};


char getch();

#endif // !OFFBOARDCONTROL_H
