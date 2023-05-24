/**
 * PX4 quadrotor control node
 * 
 * Dknt 2023.5
*/

#include "px4_sim/OffboardControl.h"

#define DRONE_PREFIX "iris_0"

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

    OffboardControl offb_ctr(nh, prefix, 20, false);
    offb_ctr.start();

    
    return 0;
}
