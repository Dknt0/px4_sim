/**
 * vins Odometry to PoseStamped with rotation
 * 
 * Dknt 2023.5
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define DRONE_PREFIX "iris_0"

using std::cout;
using std::endl;

int main(int argc, char** argv) {
    /* Get prefix from user input */
    std::string prefix;
    if (argc > 1) {
        prefix = argv[1];
    }
    else {
        prefix = DRONE_PREFIX;
    }

    // 初始化节点
    ros::init(argc, argv, "vins_camera_transfer");
    ros::NodeHandle nh;
    ros::Rate rate(60);

    // 旋转四元数
    tf2::Quaternion qtn_1, qtn_2;
    qtn_1.setEulerZYX(0, -M_PI_2, M_PI_2);
    // qtn_2.setEuler(0, -M_PI_2, -M_PI_2);
    qtn_2.setEulerZYX(-M_PI_2, 0, -M_PI_2);

    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.frame_id = "world";

    // 将 Odometry 转发为 PoseStamped
    ros::Subscriber vins_sub = nh.subscribe<nav_msgs::Odometry>("vins_estimator/camera_pose", 10, 
        [&](const nav_msgs::Odometry::ConstPtr& msg) {
            camera_pose.pose.position.x = msg->pose.pose.position.x;
            camera_pose.pose.position.y = msg->pose.pose.position.y;
            camera_pose.pose.position.z = msg->pose.pose.position.z;
            tf2::Quaternion qtn_temp = tf2::Quaternion(msg->pose.pose.orientation.x,
                                                       msg->pose.pose.orientation.y,
                                                       msg->pose.pose.orientation.z,
                                                       msg->pose.pose.orientation.w);
            // 只需要相机位姿的话，这里可以不转
            // qtn_temp = qtn_temp * qtn_1;
            // qtn_temp = qtn_temp * qtn_2;
            camera_pose.pose.orientation.w = qtn_temp.getW();
            camera_pose.pose.orientation.x = qtn_temp.getX();
            camera_pose.pose.orientation.y = qtn_temp.getY();
            camera_pose.pose.orientation.z = qtn_temp.getZ();
        });
    
    ros::Publisher cam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(prefix + "/camera_pose", 10);
    
    // 发送消息
    while (ros::ok()) {
        camera_pose.header.stamp = ros::Time::now();
        cam_pose_pub.publish(camera_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
