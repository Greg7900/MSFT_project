#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

// Callback function for the QR code pose
void qrCodePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("QR Code Pose - Position: [" 
        << msg->pose.position.x << ", " 
        << msg->pose.position.y << ", " 
        << msg->pose.position.z << "], Orientation: [" 
        << msg->pose.orientation.x << ", " 
        << msg->pose.orientation.y << ", " 
        << msg->pose.orientation.z << ", " 
        << msg->pose.orientation.w << "]");

    //transform the qr_code quaternion to roll pitch yaw
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("QR Code Pose - Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw);

    //transform the qr_code quaternion to euler angles
    tf::Quaternion q2(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getEulerYPR(yaw2, pitch2, roll2);
    ROS_INFO_STREAM("QR Code Pose - euler angles - Roll: " << roll2 << ", Pitch: " << pitch2 << ", Yaw: " << yaw2);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "qr_code_pose_printer");
    ros::NodeHandle nh;

    // Subscribe to the QR code pose topic
    ros::Subscriber qr_code_sub = nh.subscribe("/visp_auto_tracker/object_position", 10, qrCodePoseCallback);

    // Spin to process callbacks
    ros::spin();

    return 0;
}
