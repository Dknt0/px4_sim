/**
 * Get model pose ground truth from gazebo,
 * because our VIO is not stable enough for path planning.
 * 
 * Dknt 2023.5
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <mutex>
#include <future>
#include <chrono>

int main(int argc, char** argv) {

    // 初始化节点
    ros::init(argc, argv, "odmo_gt_pub");
    ros::NodeHandle nh;

    // 查询模型服务 不稳定!!!!  DO NOT USE
    ros::ServiceClient states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    // 发布里程计信息
    ros::Publisher odmo_pub = nh.advertise<nav_msgs::Odometry>("/odmo_2d", 20);


    // tf pub
    static tf::TransformBroadcaster odmo_br;
    
    tf::Transform transform;
    tf::Quaternion quaternion;
    ros::Rate rate(30);

    // var
    gazebo_msgs::GetModelState model_state;
    std::mutex m_model_state;
    model_state.request.model_name = "iris_0";
    model_state.request.relative_entity_name = "world";

    nav_msgs::Odometry odmo;

    // 从 gz 话题中查找模型位置，这个是稳定的
    ros::Subscriber states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 5,
        [&](const gazebo_msgs::ModelStates::ConstPtr& msg) {
            size_t i = 0;
            // std::cout << msg->name[0] << std::endl;
            for (auto &body: msg->name) {
                // std::cout << body << std::endl;
                if (body == "iris_0") {
                    auto model_state_pose = msg->pose[i];
                    auto model_state_twist = msg->twist[i];
                    m_model_state.lock();
                    odmo.pose.pose = model_state_pose;
                    odmo.twist.twist = model_state_twist;
                    m_model_state.unlock();
                    break;
                }
                i++;
            }
            
            // std::cout << "test " << std::endl;
            rate.sleep();
            return true;
        });

    // 开线程
    // std::thread get_odmo([&]() {
    //         while (ros::ok()) {
    //             states_client.call(model_state);
    //             m_model_state.lock();
    //             odmo.pose.pose = model_state.response.pose;
    //             // odmo.pose.pose.orientation = model_state.response.pose;
    //             odmo.twist.twist = model_state.response.twist;
    //             m_model_state.unlock();
    //             std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //             std::cout << "test " << std::endl; 
    //             ros::spinOnce();
    //         }
    //     });

    while (ros::ok()) {
        

        m_model_state.lock();
        odmo_pub.publish(odmo);
        tf::quaternionMsgToTF(odmo.pose.pose.orientation, quaternion);
        // quaternion.setW(odmo.pose.pose.orientation.w);
        // quaternion.setX(odmo.pose.pose.orientation.x);
        // quaternion.setY(odmo.pose.pose.orientation.y);
        // quaternion.setZ(odmo.pose.pose.orientation.z);
        // quaternion.normalize();
        transform.setOrigin(tf::Vector3(odmo.pose.pose.position.x,
                                        odmo.pose.pose.position.y,
                                        odmo.pose.pose.position.z));
        m_model_state.unlock();

        transform.setRotation(quaternion);
        
        // std::cout << quaternion.getW() << std::endl;

        odmo_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odmo_2d", "base_link"));

        ros::spinOnce();
        rate.sleep();
    }

    // get_odmo.join();

    return 0;
}
