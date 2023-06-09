/**
 * PX4 quadrotor control class
 * 
 * Dknt 2023.5
*/

#include "px4_sim/OffboardControl.h"

/**********************************
 * 功能函数
 * 
***********************************/

OffboardControl::OffboardControl(ros::NodeHandle &nh,
                                 std::string &prefix, 
                                 int rate,
                                 bool using_cv,
                                 std::string camera_topic):
                                 _prefix(prefix), _using_cv(using_cv), _camera_topic(camera_topic) {
    // set rate
    this->_rate = ros::Rate(rate);
    this->init(nh);
}

// start
void OffboardControl::start() {
    // 开线程 图像处理与显示
    std::shared_ptr<std::thread> img_display;
    if (_using_cv) {
        img_display.reset(new std::thread(p_img_display, static_cast<void*>(this)));
    }

    this->arm();
    this->offboard();

    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    // cout << "sleeping" << endl;

    this->takeoff(2.5);
    
    // this->keyboard_vel_ctr();

    path_planning(0, -40, 0);

    
    /* 战术悬停 */
    // 悬停单写一个函数
    // 悬停   好神奇，竟然能用
    std::promise<bool> prom = std::promise<bool>();
    std::future<bool> fut = prom.get_future();
    std::thread hover(p_hover, static_cast<void*>(this), static_cast<void*>(&fut));
    cout << "Press Enter to exit hover mode" << flush;
    cin.get(); // 随便一个触发条件
    prom.set_value(true);
    hover.join();


    this->land();
    this->disarm();

}

// init
void OffboardControl::init(ros::NodeHandle &nh) {
    /* pubs */
    // position control setpoint publisher
    this->_setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(_prefix + "/mavros/setpoint_position/local", 10);
    // velocity control setpoint publisher
    // this->_setpoint_row_pub = nh.advertise<geometry_msgs::Twist>(_prefix + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    this->_setpoint_row_pub = nh.advertise<mavros_msgs::PositionTarget>(_prefix + "/mavros/setpoint_raw/local", 10);
    // movebase target publisher
    this->_movebase_target_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    /* subs */
    // drone state subscriber
    this->_state_sub = nh.subscribe<mavros_msgs::State>(_prefix + "/mavros/state", 10,
        [&](const mavros_msgs::State::ConstPtr& msg) -> bool {
            _drone_state = *msg;
            return true;
        });
    // pose subscriber
    this->_loc_pos_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(_prefix + "/mavros/local_position/pose", 10,
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            _loc_pos_pose = *msg;
            return true;
        }); // 监听位置
    // velocity subscriber
    this->_loc_pos_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(_prefix + "/mavros/local_position/velocity", 10,
        [&](const geometry_msgs::TwistStamped::ConstPtr& msg) {
            _loc_pos_vel = *msg;
            return true;
        }); // 监听速度
    // image subscriber
    if (_using_cv) {
        this->_loc_pos_vel_sub = nh.subscribe<sensor_msgs::Image>(_prefix + _camera_topic, 10,
            [&](const sensor_msgs::Image::ConstPtr& msg) {
                m_img.lock();
                _img = *msg;
                m_img.unlock();
                
                return true;
            });
    }
    // path planning subscriber
    this->_loc_pos_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg) {
            _cmd_vel = *msg;
            return true;
        }); // 监听导航指令

    /* clients */
    // arming client
    this->_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(_prefix + "/mavros/cmd/arming");
    // set mode client
    this->_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(_prefix + "/mavros/set_mode");
    // land client
    this->_land_client = nh.serviceClient<mavros_msgs::CommandTOL>(_prefix + "/mavros/cmd/land");
    /* Action clients */
    // this->_move_base_client.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
    
    

    /* wait for connection */
    ROS_INFO("Waiting for connection...");
    while (ros::ok && !_drone_state.connected) {
        ros::spinOnce();
        _rate.sleep();
    }
    ROS_INFO("Connected.");

}

/**********************************
 * 回调函数
 * 
***********************************/



/**********************************
 * 动作
 * 
***********************************/

// 切换外部控制模式
void OffboardControl::offboard() {
    /* change mode to offboard control */
    ROS_INFO("Changing to offboard mode.");
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = 0; // using FLU coordinate frame

    // before calling a request, need to send some setpoints
    for (size_t i = 0; i < 50; i++) {
        _setpoint_pos_pub.publish(setpoint);
        ros::spinOnce();
        _rate.sleep();
    }

    mavros_msgs::SetMode setModeReq;
    setModeReq.request.custom_mode = "OFFBOARD";
    _set_mode_client.call(setModeReq);
    
    if (!setModeReq.response.mode_sent) {
        ROS_ERROR("Failed to set offboard mode.");
    }
    else {
        ROS_INFO("Offboard mode sent.");
    }
}

// 解锁无人机
void OffboardControl::arm() {
    /* arm the vehicle */
    ROS_INFO("Armming...");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    _arming_client.call(arm_cmd);

    if (!arm_cmd.response.success) {
        ROS_ERROR("Failed to arm.");
    }
    else {
        ROS_INFO("Armed.");
    }
}

// 上锁
void OffboardControl::disarm() {
    /* disarm the vehicle */
    ROS_INFO("Disarmming...");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    _arming_client.call(arm_cmd);

    if (!arm_cmd.response.success) {
        ROS_ERROR("Failed to disarm.");
    }
    else {
        ROS_INFO("Disarmed.");
    }
}

// 芜湖起飞 外部控制模式实现
// 尝试调用服务实现
void OffboardControl::takeoff(double altitude) {
    geometry_msgs::PoseStamped setpoint;
    // setpoint.pose.orientation. = 0;
    setpoint.pose = _loc_pos_pose.pose;
    setpoint.pose.position.z = altitude; // using FLU coordinate frame

    ROS_INFO("Taking off, target altitude %f...", altitude);
    while (ros::ok()) {
        _setpoint_pos_pub.publish(setpoint);
        std::cout << "\rCurrent altitude " << _loc_pos_pose.pose.position.z << "...\t\t" << std::flush;
        if (abs(altitude - _loc_pos_pose.pose.position.z) < 0.10) {
            break;
        }
        ros::spinOnce();
        _rate.sleep();
    }

    std::cout << std::endl;
    ROS_INFO("Taken off.");
}

// 降落
void OffboardControl::land() {
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    _land_client.call(land_cmd);
    
    if (!land_cmd.response.success) {
        ROS_ERROR("Failed to set land mode.");
        ROS_ERROR("%d", land_cmd.response.result);
        return;
    }
    else {
        ROS_INFO("Landing...");
    }

    while (ros::ok()) {
        std::cout << "\rCurrent altitude " << _loc_pos_pose.pose.position.z << "...\t\t" << std::flush;
        if (abs(_loc_pos_pose.pose.position.z) < 0.05) {
            break;
        }
        ros::spinOnce();
        _rate.sleep();
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3)); // wait for landing
    ROS_INFO("Landed.");
}

// 键盘控制
void OffboardControl::keyboard_vel_ctr() {
    // kb_get 中的 promise，用于退出键盘控制
    std::promise<bool> prom = std::promise<bool>();
    std::future<bool> fut = prom.get_future();
    ROS_INFO("Keyboard control mode.");

    // 开线程
    std::thread keyboard_vel_get(p_keyboard_vel_get, static_cast<void*>(this), static_cast<void*>(&prom));

    // ...
    while (ros::ok() && fut.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout) {
        m_keyb_vel_setpoint.lock();
        _setpoint_row_pub.publish(keyboard_vel_setpoint);
        m_keyb_vel_setpoint.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        ros::spinOnce();
    }

    fut.get();
    keyboard_vel_get.join();

    ROS_INFO("End of keyboard control mode.");
}

// Path planning function
void OffboardControl::path_planning(double _x, double _y, double _yaw) {
    ROS_INFO("Start path planning.");

    /* send target to move_base */
    // target
    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    target.pose.position.x = _x;
    target.pose.position.y = _y;

    tf2::Quaternion qtn;
    qtn.setEulerZYX(_yaw, 0, 0);

    target.pose.orientation.w = qtn.getW();
    target.pose.orientation.x = qtn.getX();
    target.pose.orientation.y = qtn.getY();
    target.pose.orientation.z = qtn.getZ();

    for (size_t i = 0; i < 3; ++i) {
        _movebase_target_pub.publish(target);
    }

    /* listen to move_base command */
    // init
    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.linear.z = 0;
    _cmd_vel.angular.x = 0;
    _cmd_vel.angular.y = 0;
    _cmd_vel.angular.z = 0;

    // set frame FRD
    keyboard_vel_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    // set mask velocity mode
    keyboard_vel_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX  | mavros_msgs::PositionTarget::IGNORE_PY  |
                                      mavros_msgs::PositionTarget::IGNORE_PZ  | mavros_msgs::PositionTarget::IGNORE_AFX |
                                      mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                      mavros_msgs::PositionTarget::IGNORE_YAW;
    // reset vel
    keyboard_vel_setpoint.velocity.x = 0;
    keyboard_vel_setpoint.velocity.y = 0;
    keyboard_vel_setpoint.velocity.z = 0;
    keyboard_vel_setpoint.yaw_rate = 0;

    // err to target
    double err = 100;

    while (ros::ok() && err > 0.5) {
        keyboard_vel_setpoint.velocity.x = _cmd_vel.linear.x;
        keyboard_vel_setpoint.velocity.y = _cmd_vel.linear.y;
        keyboard_vel_setpoint.velocity.z = _cmd_vel.linear.z;
        keyboard_vel_setpoint.yaw_rate = _cmd_vel.angular.z;
        
        _setpoint_row_pub.publish(keyboard_vel_setpoint);
        ros::spinOnce();
        
        auto err_x = _x - _loc_pos_pose.pose.position.x - INIT_POS_X;
        auto err_y = _y - _loc_pos_pose.pose.position.y - INIT_POS_Y;

        err = sqrt(pow(err_x, 2.0) + pow(err_y, 2.0));
        // 输出误差
        std::cout.precision(2);
        std::cout << "\rCurrent position:" << RED << " x " << _loc_pos_pose.pose.position.x
                                           << BLUE << " y " << _loc_pos_pose.pose.position.y
                                           << YELLOW << " err " << err << RESET << "           " << std::flush;
        std::cout.precision(6);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    std::cout << std::endl;

    // move_base.join();

    ROS_INFO("End of path planning.");
}

// commination with move_base
// void OffboardControl::set_move_base_target(double _x, double _y, double _yaw) {


//     std::cout << "\rCurrent target speed:" << RED <<" x " << _x
//                                            << GREEN << "\t y " << _y
//                                            << BLUE << "\t yaw " << _yaw
//                                            << RESET << std::endl;;
//     move_base_msgs::MoveBaseGoal goal;

//     // have 2 time stamps
//     goal.target_pose.header.frame_id = "map";

//     goal.target_pose.pose.position.x = _x;
//     goal.target_pose.pose.position.y = _y;
//     goal.target_pose.pose.position.z = 0.0;

//     // transform Euler to Quaternion
//     tf2::Quaternion qtn;
//     qtn.setEulerZYX(_yaw, 0, 0);

//     goal.target_pose.pose.orientation.x = qtn.getX();
//     goal.target_pose.pose.orientation.y = qtn.getY();
//     goal.target_pose.pose.orientation.z = qtn.getZ();
//     goal.target_pose.pose.orientation.w = qtn.getW();

//     // 开始执行导航
//     this->_planning = true;

//     this->_move_base_client->sendGoal(goal);
//     this->_move_base_client->waitForResult();
    
//     // 结束执行导航
//     this->_planning = false;

//     if (this->_move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//         ROS_INFO("Reached goal!");
//     }
//     else {
//         ROS_ERROR("Failed to reach goal!");
//     }

// }

/**********************************
 * 静态函数
 * 
***********************************/

// 悬停模式
// 静态函数 作为线程函数使用
void OffboardControl::p_hover(void *_this, void *_fut) {
    OffboardControl *ofc = static_cast<OffboardControl*>(_this);
    std::future<bool> *fut = static_cast<std::future<bool>*>(_fut);
    
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position = ofc->_loc_pos_pose.pose.position; // using FLU coordinate frame
    setpoint.pose.orientation = ofc->_loc_pos_pose.pose.orientation;

    // ROS_INFO("Hover mode.");

    while (ros::ok() && fut->wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout) {
        ofc->_setpoint_pos_pub.publish(setpoint);
        ros::spinOnce();
    }
    
    fut->get();
    // ROS_INFO("Exiting hover mode.");
    return;
}

// 速度控制模式键值获取
// 静态函数 作为线程函数使用
void OffboardControl::p_keyboard_vel_get(void *_this, void *_pro) {
    OffboardControl *ofc = static_cast<OffboardControl*>(_this);
    std::promise<bool> *pro = static_cast<std::promise<bool>*>(_pro);

    // set frame FRD
    ofc->keyboard_vel_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    // set mask velocity mode
    ofc->keyboard_vel_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX  | mavros_msgs::PositionTarget::IGNORE_PY  |
                                           mavros_msgs::PositionTarget::IGNORE_PZ  | mavros_msgs::PositionTarget::IGNORE_AFX |
                                           mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                           mavros_msgs::PositionTarget::IGNORE_YAW;
    // reset vel
    ofc->keyboard_vel_setpoint.velocity.x = 0;
    ofc->keyboard_vel_setpoint.velocity.y = 0;
    ofc->keyboard_vel_setpoint.velocity.z = 0;
    ofc->keyboard_vel_setpoint.yaw_rate = 0;

    char key;
    bool exit_flag = false;

    while (ros::ok() && !exit_flag) {
        // 接受输入
 
        key = getch();
        
        
        std::cout << "\rget: " << key << std::flush;

        ofc->m_keyb_vel_setpoint.lock();

        // reset vel
        ofc->keyboard_vel_setpoint.velocity.x = 0;
        ofc->keyboard_vel_setpoint.velocity.y = 0;
        ofc->keyboard_vel_setpoint.velocity.z = 0;
        ofc->keyboard_vel_setpoint.yaw_rate = 0;
        
        // 处理速度 NED frame
        switch (key) {
            /* hold mode */
            case 's':
            case 'k':
                {
                    // 悬停   好神奇，竟然还能在这里用
                    std::promise<bool> prom = std::promise<bool>();
                    std::future<bool> fut = prom.get_future();
                    std::thread hover(ofc->p_hover, static_cast<void*>(ofc), static_cast<void*>(&fut));
                    key = getch();
                    prom.set_value(true);
                    hover.join();
                    break;
                }

            /* left */
            case 'w':
                ofc->keyboard_vel_setpoint.velocity.z = VELOCITY_UP;
                break;
            
            case 'x':
                ofc->keyboard_vel_setpoint.velocity.z = -VELOCITY_UP;
                break;
            
            case 'a':
                ofc->keyboard_vel_setpoint.yaw_rate = VELOCITY_YAW;
                break;
            
            case 'd':
                ofc->keyboard_vel_setpoint.yaw_rate = -VELOCITY_YAW;
                break;
                
            /* right */
            case 'i':
                ofc->keyboard_vel_setpoint.velocity.x = VELOCITY_FORWARD;
                break;

            case ',':
                ofc->keyboard_vel_setpoint.velocity.x = -VELOCITY_FORWARD;
                break;

            case 'j':
                ofc->keyboard_vel_setpoint.velocity.y = VELOCITY_LEFT;
                break;

            case 'l':
                ofc->keyboard_vel_setpoint.velocity.y = -VELOCITY_LEFT;
                break;

            case 'u':
                ofc->keyboard_vel_setpoint.velocity.x = VELOCITY_FORWARD;
                ofc->keyboard_vel_setpoint.velocity.y = VELOCITY_LEFT;
                break;

            case 'o':
                ofc->keyboard_vel_setpoint.velocity.x = VELOCITY_FORWARD;
                ofc->keyboard_vel_setpoint.velocity.y = -VELOCITY_LEFT;
                break;

            case 'm':
                ofc->keyboard_vel_setpoint.velocity.x = -VELOCITY_FORWARD;
                ofc->keyboard_vel_setpoint.velocity.y = VELOCITY_LEFT;
                break;

            case '.':
                ofc->keyboard_vel_setpoint.velocity.x = -VELOCITY_FORWARD;
                ofc->keyboard_vel_setpoint.velocity.y = -VELOCITY_LEFT;
                break;
            
            // 单位方向保持...

            // ESC 退出
            case 27:
                exit_flag = true;
                pro->set_value(true);
                break;
            
            default:
                break;
        }
        ofc->m_keyb_vel_setpoint.unlock();
    }
}

// 图像处理与显示
// 静态函数 线程函数
void OffboardControl::p_img_display(void *_this) {
    OffboardControl *ofc = static_cast<OffboardControl*>(_this);

    // create new window
    cv::namedWindow("Offboard Control", cv::WINDOW_GUI_EXPANDED);
    cv::resizeWindow("Offboard Control", cv::Size(752, 480));

    while (ros::ok()) {
        // img message to cv Mat
        ofc->m_img.lock();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ofc->_img, sensor_msgs::image_encodings::TYPE_8UC3);
        ofc->m_img.unlock();

        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);

        cv::imshow("Offboard Control", cv_ptr->image);
        char key = cv::waitKey(1);
    }

}



/**********************************
 * 工具函数
 * 
***********************************/

// 基于终端的键盘控制
// 神奇的字符获取函数
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}
