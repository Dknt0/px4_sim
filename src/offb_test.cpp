#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#define DRONE_PREFIX "iris_0"

// using std::cout, std::endl, std::flush;

int main(int argc, char** argv) {
    /* Get prefix from user input */
    std::string prefix;
    if (argc > 1) {
        prefix = argv[1];
    }
    else {
        prefix = DRONE_PREFIX;
    }

    /* ROS init */
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // drone state subscriber
    mavros_msgs::State drone_state;
    auto state_sub = nh.subscribe<mavros_msgs::State>(prefix + "/mavros/state", 10,
        [&drone_state](const mavros_msgs::State::ConstPtr& msg){
            drone_state = *msg;
        });
    // setpoint publisher
    auto setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(prefix + "/mavros/setpoint_position/local", 10);
    // arming client
    auto arming_client = nh.serviceClient<mavros_msgs::CommandBool>(prefix + "/mavros/cmd/arming");
    // set mode client
    auto set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(prefix + "/mavros/set_mode");

    ros::Rate rate(20);

    /* wait for connection */
    while (ros::ok && !drone_state.connected) {
        ROS_INFO("Waiting for connection...");
        ros::spinOnce();
        rate.sleep();
    }

    /* change mode to offboard control */
    ROS_INFO("Changing to offboard mode.");
    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = 2; // using FLU coordinate frame

    // before calling a request, need to send some setpoints
    for (size_t i = 0; i < 100; i++) {
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode setModeReq;
    setModeReq.request.custom_mode = "OFFBOARD";
    set_mode_client.call(setModeReq);
    
    if (!setModeReq.response.mode_sent) {
        ROS_ERROR("Failed to set offboard mode, quitting.");
        return 1;
    }
    ROS_INFO("Changed to offboard mode.");

    /* arm the vehicle */
    ROS_INFO("Armming...");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    arming_client.call(arm_cmd);

    if (!arm_cmd.response.success) {
        ROS_ERROR("Failed to arm, quitting.");
        return 1;
    }
    ROS_INFO("Armed.");

    while (ros::ok()) {
        
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
