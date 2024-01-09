// gazebo_cube_node.cpp

#include "doosan_gazebo_sim/gazebo_cube.h"



int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_cube_node");
    GazeboCube robot_mover("doosan_arm"); // Replace with your move group name
    robot_mover.execute();
    return 0;
}