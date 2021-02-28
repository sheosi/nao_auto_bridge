#include "nao_auto_bridge.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <naoqi_bridge_msgs/CmdVelService.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <naoqi_bridge_msgs/CmdPoseService.h>
#include <std_srvs/Empty.h>
#include <naoqi_bridge_msgs/SetArmsEnabled.h>


// Subscribers
void SimulatedNao::OnCmdVel(const geometry_msgs::Twist &msg) {
    //Note: We could replicate behaviour
    ROS_INFO("Received cmd_vel command \"linear{x: %f, y: %f, z: %f}, angular{x: %f, y: %f, z: %f}\"",msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x, msg.angular.y, msg.angular.z);
}

void SimulatedNao::OnCmdPose(const geometry_msgs::Pose2D &msg) {
    //Note: We could replicate behaviour
    ROS_INFO("Received cmd_pose command \"x: %f, y: %f, theta: %f\"", msg.x, msg.y, msg.theta);
}

void SimulatedNao::OnCmdStep(const humanoid_nav_msgs::StepTarget &msg) {
    //Note: We could replicate behaviour
    ROS_INFO("Received cmd_step command \"pose{x: %f, y: %f, theta:%f}, leg:%u\"", msg.pose.x, msg.pose.y, msg.pose.theta, msg.leg);
}

// Services
void SimulatedNao::OnReadFootGaitConfigSrv() {
    //Note: We could replicate behaviour
    ROS_INFO("Asked to read ~use_foot_gait_config ~foot_gait_config");
}

void SimulatedNao::OnCmdVelSrv(const geometry_msgs::Twist &msg) {
    //Note: We could replicate behaviour
    this->OnCmdVel(msg);
}

void SimulatedNao::OnCmdStepSrv(const humanoid_nav_msgs::StepTarget &msg) {
    //Note: We could replicate behaviour
    this->OnCmdStep(msg);
}

void SimulatedNao::OnCmdPoseSrv(const geometry_msgs::Pose2D &msg) {
    //Note: We could replicate behaviour
    this->OnCmdPose(msg);
}

void SimulatedNao::OnStopWalkSrv() {
    //Note: We could replicate behaviour
    ROS_INFO("Asked to stop walking");
}

void SimulatedNao::OnNeedsStartWalkPoseSrv() {
    //Note: We could replicate behaviour
    ROS_INFO("Asked to make sure to have a suitable pose before walking");
}

void SimulatedNao::OnEnableArmWalkingSrv(bool left, bool right) {
    //Note: We could replicate behaviour
    if (left) {
        ROS_INFO("Left arm movement during walk enabled");    
    }
    else {
        ROS_INFO("Left arm movement during walk disabled");
    }

    if (right) {
        ROS_INFO("Right arm movement during walk enabled");
    }
    else {
        ROS_INFO("Right arm movement during walk disabled");
    }
}


namespace BridgeNaoWalker {
    ros::Subscriber sub_cmd_vel,
                    sub_cmd_pose,
                    sub_cmd_step;

    ros::ServiceServer srv_read_foot_gait_config_srv,
                       srv_cmd_vel_srv,
                       srv_cmd_step_srv,
                       srv_cmd_pose_srv,
                       srv_stop_walk_srv,
                       srv_needs_start_walk_pose_srv,
                       srv_enable_arms_walking_srv;

        // Subscriptions
    void on_cmd_vel(
        const geometry_msgs::Twist &msg
    ) {
        nao_connection.OnCmdVel(msg);
    }

    void on_cmd_pose(
        const geometry_msgs::Pose2D &msg
    ) {
        nao_connection.OnCmdPose(msg);
    }

    void on_cmd_step(
        const humanoid_nav_msgs::StepTarget &msg
    ) {
        nao_connection.OnCmdStep(msg);
    }

    // Services
    bool on_read_foot_gait_config_srv(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {
         nao_connection.OnReadFootGaitConfigSrv();
         return true;
    }

    bool on_cmd_vel_srv(
        naoqi_bridge_msgs::CmdVelService::Request &req,
        naoqi_bridge_msgs::CmdVelService::Response &resp
    ) {
        nao_connection.OnCmdVelSrv(req.twist);
        return true;
    }

    bool on_cmd_step_srv(
        humanoid_nav_msgs::StepTargetService::Request &req,
        humanoid_nav_msgs::StepTargetService::Response &resp
    ) {
        nao_connection.OnCmdStepSrv(req.step);
        return true;
    }

    bool on_cmd_pose_srv(
        naoqi_bridge_msgs::CmdPoseService::Request &req,
        naoqi_bridge_msgs::CmdPoseService::Response &resp
    ) {
        nao_connection.OnCmdPoseSrv(req.pose);
        return true;
    }

    bool on_stop_walk_srv(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {
        nao_connection.OnStopWalkSrv();
        return true;
    }

    bool on_needs_start_walk_pose_srv(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {
        nao_connection.OnNeedsStartWalkPoseSrv();
        return true;
    }

    bool on_enable_arms_walking_srv(
        naoqi_bridge_msgs::SetArmsEnabled::Request &req,
        naoqi_bridge_msgs::SetArmsEnabled::Response &resp
    ) {
        nao_connection.OnEnableArmWalkingSrv(req.left_arm, req.right_arm);
        return true;
    }

    void init(ros::NodeHandle &n) {
        double step_frequency;
        bool use_walk_pose, enable_foot_contact_protection;
        float init_stiffness;
        bool use_foot_gait_config;

        n.param("step_frequency", step_frequency, 0.5);
        n.param("use_walk_pose", use_walk_pose, false);
        n.param("enable_foot_contact_protection", enable_foot_contact_protection, false);
        n.param("use_foot_gait_config", use_foot_gait_config, false);
        //n.param("foot_gait_config", ,); // Note: NYI
        n.param("init_stiffness", init_stiffness, 0.0f);

        sub_cmd_vel = n.subscribe("cmd_vel", 1000, on_cmd_vel);
        sub_cmd_pose = n.subscribe("cmd_pose", 1000, on_cmd_pose);
        sub_cmd_step = n.subscribe("cmd_step", 1000, on_cmd_step);

        srv_read_foot_gait_config_srv = n.advertiseService("read_foot_gait_config_srv", on_read_foot_gait_config_srv);
        srv_cmd_vel_srv = n.advertiseService("cmd_vel_srv", on_cmd_vel_srv);
        srv_cmd_step_srv = n.advertiseService("cmd_step_srv", on_cmd_step_srv);
        srv_cmd_pose_srv = n.advertiseService("cmd_pose_srv", on_cmd_pose_srv);
        srv_stop_walk_srv = n.advertiseService("stop_walk_srv", on_stop_walk_srv);
        srv_needs_start_walk_pose_srv = n.advertiseService("needs_start_walk_pose_srv", on_needs_start_walk_pose_srv);
        srv_enable_arms_walking_srv = n.advertiseService("enable_arms_walking_srv", on_enable_arms_walking_srv);
    }
}
