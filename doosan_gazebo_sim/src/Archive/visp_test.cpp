#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureBuilder.h>




class VisualServoingControl {
private:
    ros::NodeHandle nh_;
    ros::Subscriber subPose_;
    ros::Subscriber subImage_;
    ros::Subscriber subCameraInfo_;
    moveit::planning_interface::MoveGroupInterface move_group;
    vpImage<unsigned char> vispImage_;
    vpDisplayX display_;
    vpServo task;
    vpFeaturePoint s[4], sd[4];
    vpCameraParameters cam_;
    vpHomogeneousMatrix cMo;
    vpPoint point[4];

public:
    VisualServoingControl(const std::string& move_group_name)
        : move_group(move_group_name){
        subPose_ = nh_.subscribe("/visp_auto_tracker/object_position", 1000, &VisualServoingControl::poseCallback, this);
        subImage_ = nh_.subscribe("/robotic_arm/camera1/image_raw", 1000, &VisualServoingControl::imageCallback, this);
        subCameraInfo_ = nh_.subscribe("/camera_info", 1000, &VisualServoingControl::cameraInfoCallback, this);
        vispImage_.resize(480, 640); // Set according to your camera's resolution
        initVisualServo();
    }

    void initVisualServo();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void convertVelocityToRobotCommands(const vpColVector &v);
};

void VisualServoingControl::initVisualServo() {
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    cam_.initPersProjWithoutDistortion(800, 795, 320, 216); // Example parameters

    double L = 0.05; // Size of the square
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates(L, -L, 0);
    point[2].setWorldCoordinates(L, L, 0);
    point[3].setWorldCoordinates(-L, L, 0);

    for (int i = 0; i < 4; ++i) {
        point[i].track(cMo); // Update for current pose
        vpFeatureBuilder::create(sd[i], point[i]);
        task.addFeature(s[i], sd[i]);
    }
}

void VisualServoingControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    for (int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(s[i], point[i]);
    }

    vpColVector v = task.computeControlLaw();
    convertVelocityToRobotCommands(v);

    if (vispImage_.getWidth() != 0 && vispImage_.getHeight() != 0) {
        vpDisplay::display(vispImage_);
        // Additional display...
        vpDisplay::flush(vispImage_);
    }
}

void VisualServoingControl::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        vispImage_ = visp_bridge::toVispImage(*msg);
        if (!display_.isInitialised()) {
            display_.init(vispImage_, 100, 100, "Robot Camera View");
        }
        vpDisplay::display(vispImage_);
        vpDisplay::flush(vispImage_);
    } catch (...) {
        ROS_ERROR("Exception caught during processing of the image.");
    }
}

void VisualServoingControl::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    cam_ = visp_bridge::toVispCameraParameters(*msg);
    ROS_INFO("Camera info received.");
}

void VisualServoingControl::convertVelocityToRobotCommands(const vpColVector &v) {
    // Convert the visual servoing velocity command to robot commands
    // This might involve inverse kinematics or other transformations
    // Example (placeholder logic):
    // geometry_msgs::Pose target_pose;
    // target_pose.position.x += v[0];
    // target_pose.position.y += v[1];
    // target_pose.position.z += v[2];
    // move_group.setPoseTarget(target_pose);
    // move_group.move();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_servoing_control");
    VisualServoingControl vsc("doosan_arm"); // Replace with your MoveGroup name
    ros::spin();
    return 0;
}
