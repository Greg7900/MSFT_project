// gazebo_cube.cpp

#include "doosan_gazebo_sim/gazebo_cube.h"


GazeboCube::GazeboCube(const std::string& move_group_name)
    : move_group(move_group_name), spinner(1), nh("~") {
    spinner.start();
    move_group.setPlannerId("BiTRRT");
    move_group.setPlanningTime(5.0);  // Planning time set to 5 seconds
    move_group.setNumPlanningAttempts(10);  // Set number of planning attempts
    move_group.setMaxVelocityScalingFactor(0.30);
    move_group.setMaxAccelerationScalingFactor(0.30);
    target_pose.orientation.x = 1;
    last_target_pose.orientation.x = 1; // Adjust as per your initial orientation   






}

void GazeboCube::startTracking() {
    qr_code_sub = nh.subscribe("/visp_auto_tracker/object_position", 10, &GazeboCube::qrCodePoseCallback, this);
}

    //ROS_INFO_STREAM("QR Code Pose: " << "Position: [" << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << "], Orientation: [" << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w << "]");


void GazeboCube::qrCodePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Get the current pose of the robot's end-effector
    geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose();
    geometry_msgs::Pose current_pose = current_pose_stamped.pose;

    // Position adjustment to keep the QR code centered

    double adjustment_factor = 0.3; // Adjust this factor as needed for finer control
    double optimal_z_distance = 0.118; // Optimal distance from the QR code in meters
    target_pose.position.x = current_pose.position.x + (msg->pose.position.y < 0 ? 1 : -1) * adjustment_factor * abs(msg->pose.position.y);
    target_pose.position.y = current_pose.position.y + (msg->pose.position.x < 0 ? 1 : -1) * adjustment_factor * abs(msg->pose.position.x);
    
    // Adjust z position based on QR code's z distance
    if (msg->pose.position.z < optimal_z_distance) {
        // QR code is too close, move the robot up
        target_pose.position.z = current_pose.position.z + (optimal_z_distance - msg->pose.position.z);
    } else {
        // QR code is too far, move the robot down
        target_pose.position.z = current_pose.position.z - (msg->pose.position.z - optimal_z_distance);
    }
    // Orientation adjustment (example: aligning yaw with the QR code)
    // This is a simplified example and may need to be tailored to your specific requirements

    // transform the robot quaternion to roll pitch yaw
    tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Current Pose - Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw);

    // transform the QR code quaternion to roll pitch yaw
    tf::Quaternion q2(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getRPY(roll2, pitch2, yaw2);
    ROS_INFO_STREAM("QR Code Pose - Roll: " << roll2 << ", Pitch: " << pitch2 << ", Yaw: " << yaw2);

 

    // Convert the last target pose to a tf::Quaternion
    tf::Quaternion last_q(last_target_pose.orientation.x, last_target_pose.orientation.y,
                          last_target_pose.orientation.z, last_target_pose.orientation.w);

    // New target orientation from QR code
    
    tf::Quaternion new_q(roll2, pitch2, yaw2); // Creating quaternion from RPY

    // Interpolation factor (0.0 to 1.0)
    double alpha = 0.2; // Adjust this for smoother transitions

    // Slerp interpolation for smoother orientation transition
    tf::Quaternion interpolated_q = last_q.slerp(new_q, alpha);

    target_pose.orientation.x = interpolated_q.x();
    target_pose.orientation.y = interpolated_q.y();
    target_pose.orientation.z = interpolated_q.z();
    target_pose.orientation.w = interpolated_q.w();

    // Update last_target_pose for next iteration
    last_target_pose.orientation.x = interpolated_q.x();
    last_target_pose.orientation.y = interpolated_q.y();
    last_target_pose.orientation.z = interpolated_q.z();
    last_target_pose.orientation.w = interpolated_q.w();

    // // set the robot RPY to the QR code RPY, with roll roll2, pitch pitch2, and yaw yaw2. add something to smooth the transition
    // tf::Quaternion q3(roll2, pitch2, yaw2);
    // tf::Quaternion smooth_q = q.slerp(q3, 0.2);
    // target_pose.orientation.x = smooth_q.x();
    // target_pose.orientation.y = smooth_q.y();
    // target_pose.orientation.z = smooth_q.z();
    // target_pose.orientation.w = smooth_q.w();

    // // set the robot RPY to the QR code RPY, with roll roll2, pitch pitch2, and yaw yaw2.
    // tf::Quaternion q3 = tf::createQuaternionFromRPY(-roll2, pitch2, yaw2); // -roll2 because of the camera orientation
    // target_pose.orientation.x = q3.x();
    // target_pose.orientation.y = q3.y();
    // target_pose.orientation.z = q3.z();
    // target_pose.orientation.w = q3.w();

    // Update the robot's target position and orientation
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        // Convert RobotState message to RobotState object
        moveit::core::RobotStatePtr end_state = move_group.getCurrentState();
        moveit::core::robotStateMsgToRobotState(my_plan.start_state_, *end_state);

        // Access the planning scene for collision checking
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
        planning_scene_monitor->startSceneMonitor();
        planning_scene_monitor->requestPlanningSceneState();
        planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();

        if (end_state->satisfiesBounds() && !planning_scene->isStateColliding(*end_state, "doosan_arm")) {
            move_group.execute(my_plan);
        } else {
            ROS_WARN("Planned path is in collision. Not executing.");
        }
    } else {
        ROS_WARN("Failed to plan to the target pose.");
    }

    // Debugging information
    ROS_INFO_STREAM("QR Code Pose: Position: [" << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << "], Orientation: [" << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", " << msg->pose.orientation.z << ", " << msg->pose.orientation.w << "]");
    ROS_INFO_STREAM("Current Pose: Position: [" << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << "], Orientation: [" << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z << ", " << current_pose.orientation.w << "]");
    ROS_INFO_STREAM("Updated Target Pose: Position: [" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << "] Orientation: [" << target_pose.orientation.x << ", " << target_pose.orientation.y << ", " << target_pose.orientation.z << ", " << target_pose.orientation.w << "]");
}






void GazeboCube::execute() {
    this->startTracking();

    ros::Rate loop_rate(10); // Frequency in Hz
    while (ros::ok()) {
        // Get current pose of the robot's end-effector
       
        ros::spinOnce();
        loop_rate.sleep();
    }
}