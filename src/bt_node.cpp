/**
 * PX4 quadrotor control behavior tree
 * communation with Groot
 * 
 * Dknt 2023.5
*/

#include "px4_sim/OffboardControl.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#define DRONE_PREFIX "iris_0"

using namespace BT;

using std::cout;
using std::endl;
using std::flush;
using std::cin;

const std::string tree_path = "/home/dknt/test/ros_test/src/px4_sim/tree/tree_uav.xml";

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
    ros::init(argc, argv, "offb_ctr");
    ros::NodeHandle nh;
    
    // 用于生成树
    BT::BehaviorTreeFactory factory;

    // 无人机控制类
    OffboardControl offb_ctr(nh, prefix, 20, false);

    // 起飞
    factory.registerSimpleCondition("Takeoff",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "Takeoff" << RESET << flush;

            cin.get();
            offb_ctr.arm();
            offb_ctr.offboard();
            offb_ctr.takeoff(2.5);

            return BT::NodeStatus::SUCCESS;
        });

    // 飞向树林  理论上不需要避障，可以飞快点
    factory.registerSimpleCondition("FlyToWood",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "FlyToWood" << RESET << endl;
            offb_ctr.path_planning(0, -37, 0);

            return BT::NodeStatus::SUCCESS;
        });

    // 飞往目标点1 避障
    factory.registerSimpleCondition("FlyToPosition1",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "FlyToPosition1" << RESET << endl;

            offb_ctr.path_planning(10, -30, 0);
            return BT::NodeStatus::SUCCESS;
        });

    // 飞往目标点2 避障
    factory.registerSimpleCondition("FlyToPosition2",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "FlyToPosition2" << RESET << endl;

            offb_ctr.path_planning(-10, -15, 0);
            return BT::NodeStatus::SUCCESS;
        });

    // 飞往目标点3 避障
    factory.registerSimpleCondition("FlyToPosition3",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "FlyToPosition3" << RESET << endl;

            offb_ctr.path_planning(-15, 0, 0);
            return BT::NodeStatus::SUCCESS;
        });

    // 搜寻目标  理论上由神经网络检测，实际上是我用想象力检测
    factory.registerSimpleCondition("SearchTarget",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "Searching target..." << RESET << endl;

            // 战术悬停
            std::promise<bool> prom = std::promise<bool>();
            std::future<bool> fut = prom.get_future();
            std::thread hover(offb_ctr.p_hover, static_cast<void*>(&offb_ctr), static_cast<void*>(&fut));

            cout << CYAN << "Have target? " << RESET << flush;
            int key;
            cin >> key;
            prom.set_value(true);
            hover.join();

            if (key == 0) {
                return BT::NodeStatus::FAILURE;
            }
            else {
                return BT::NodeStatus::SUCCESS;
            }
        });

    // 飞往地面载具  暂时没有地面载具
    factory.registerSimpleCondition("FlyToCar",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "FlyToCar" << RESET << endl;

            offb_ctr.path_planning(0, 0, 0);
            return BT::NodeStatus::SUCCESS;
        });

    // 降落  理论上应该有视觉伺服精准降落，现在是随便落
    factory.registerSimpleCondition("Land",
        [&](BT::TreeNode& self) {
            cout << YELLOW << "Landing" << RESET << endl;

            offb_ctr.land();
            offb_ctr.disarm();
            return BT::NodeStatus::SUCCESS;
        });

    auto tree = factory.createTreeFromFile(tree_path);

    // for Groot
    PublisherZMQ publisher_zmq(tree);

    // 连续运行
    tree.tickRootWhileRunning();
    
    return 0;
}
