#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "move_base_target");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _move_base_client("move_base", true);

    double _x = 0;
    double _y = -20;
    double _yaw = 0;

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = _x;
    goal.target_pose.pose.position.y = _y;
    goal.target_pose.pose.position.z = 0.0;

    // transform Euler to Quaternion
    tf2::Quaternion qtn;
    qtn.setEulerZYX(_yaw, 0, 0);

    goal.target_pose.pose.orientation.x = qtn.getX();
    goal.target_pose.pose.orientation.y = qtn.getY();
    goal.target_pose.pose.orientation.z = qtn.getZ();
    goal.target_pose.pose.orientation.w = qtn.getW();

    // std::cout << "\rCurrent target speed:" << RED <<" x " << _x
    //                                        << GREEN << "\t y " << _y
    //                                        << BLUE << "\t yaw " << _yaw
    //                                        << RESET << std::endl;;

    _move_base_client.sendGoal(goal);
    _move_base_client.waitForResult();

    if (_move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached goal!");
    }
    else {
        ROS_ERROR("Failed to reach goal!");
    }



    return 0;
}