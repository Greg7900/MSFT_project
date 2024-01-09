// gazebo_cube.h

#ifndef GAZEBO_CUBE_H
#define GAZEBO_CUBE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_datatypes.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <cmath> // For atan2
class GazeboCube {
public:
    GazeboCube(const std::string& move_group_name);
    void execute();
    void startTracking();
    void qrCodePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    ros::AsyncSpinner spinner;
    ros::NodeHandle nh;
    ros::Subscriber qr_code_sub;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose last_target_pose; // Member variable to store the last target pose
};

#endif // GAZEBO_CUBE_H
