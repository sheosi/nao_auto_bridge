#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <humanoide_nav_msgs/StepTarget.h>
#include <naoqi_msgs/CmdVelService.h>
#include <naoqi_msgs/StepTargetService.h>
#include <naoqi_msgs/CmdPoseSerivce.h>
#include <std_srvs/Empty.h>
#include <naoqi_msgs/SetArmsEnabled.h>

class SimulatedNao {

    // Subscribers
    void OnCmdVel() {
        //Note: We could replicate behaviour
    }

    void OnCmdPose() {
        //Note: We could replicate behaviour
    }

    void OnCmdStep() {
        //Note: We could replicate behaviour
    }

    // Services
    void OnReadFootGaitConfigSrv() {
        //Note: We could replicate behaviour
    }

    void OnCmdVelSrv() {
        //Note: We could replicate behaviour
    }

    void OnCmdStepSrv() {
        //Note: We could replicate behaviour
    }

    void OnCmdPoseSrv() {
        //Note: We could replicate behaviour
    }

    void OnStopWalkSrv() {
        //Note: We could replicate behaviour
    }

    void OnNeedsStartWalkPoseSrv() {
        //Note: We could replicate behaviour
    }

    void OnEnableArmWalkingSrv() {
        //Note: We could replicate behaviour
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

    void init(ros::NodeHandle &n) {
        double step_frequency;
        bool use_walk_pose, enable_foot_contact_protection;
        float init_stiffness;

        n.param("step_frequency", step_frequency, 0.5);
        n.param("use_walk_pose", use_walk_pose, false);
        n.param("enable_foot_contact_protection", enable_foot_contact_protection, false);
        n.param("use_foot_gait_config", use_foot_gait_config, false);
        //n.param("foot_gait_config", ,); // Note: NYI
        n.param("init_stiffness", init_stiffness, 0);

        sub_cmd_vel = n.subscribe("cmd_vel", 1000, on_cmd_vel);
        sub_cmd_pose = n.subscribe("cmd_pose", 1000, on_cmd_pose);
        sub_cmd_step = n.subscribe("cmd_step", 1000, on_cmd_step);

        srv_read_foot_gait_config_srv = n.advertiseServer("read_foot_gait_config_srv", on_read_foot_gait_config_srv);
        srv_cmd_vel_srv = n.advertiseServer("cmd_vel_srv", on_cmd_vel_srv);
        srv_cmd_step_srv = n.advertiseServer("cmd_step_srv", on_cmd_step_srv);
        srv_cmd_pose_srv = n.advertiseServer("cmd_pose_srv", on_cmd_pose_srv);
        srv_stop_walk_srv = n.advertiseServer("stop_walk_srv", on_stop_walk_srv);
        srv_needs_start_walk_pose_srv = n.advertiseServer("needs_start_walk_pose_srv", on_needs_start_walk_pose_srv);
        srv_enable_arms_walking_srv = n.advertiseServer("enable_arms_walking_srv", on_enable_arms_walking_srv);
    }

    // Subscriptions
    void on_cmd_vel(
        const geometry_msgs::TwistConstPtr &msg
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdVel();
    }

    void on_cmd_pose(
        const geometry_msgs::Pose2DConstPtr &msg
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdPose();
    }

    void on_cmd_step(
        const humanoide_nav_msgs::StepTarget &msg
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdStep();
    }

    // Services
    bool on_read_foot_gait_config_srv(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {
         std::get<SimulatedNao>(nao_connection).OnReadFootGaitConfigSrv();
         return true;
    }

    bool on_cmd_vel_srv(
        naoqi_msgs::CmdVelService::Request &req,
        naoqi_msgs::CmdVelService::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdVelSrv();
        return true;
    }

    bool on_cmd_step_srv(
        naoqi_msgs::StepTargetService::Request &req,
        naoqi_msgs::StepTargetService::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdStepSrv();
        return true;
    }

    bool on_cmd_pose_srv(
        naoqi_msgs::CmdPoseSerivce::Request &req,
        naoqi_msgs::CmdPoseSerivce::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnCmdPoseSrv();
        return true;
    }

    bool on_stop_walk_srv(
        std_srvs::Empty::Request &req,
        st::Empty::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnStopWalkSrv();
        return true;
    }

    bool on_needs_start_walk_pose_srv(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnNeedsStartWalkPoseSrv();
        return true;
    }

    bool on_enable_arms_walking_srv(
        naoqi_msgs::SetArmsEnabled::Request &req,
        naoqi_msgs::SetArmsEnabled::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnNeedsStartWalkPoseSrv();
        return true;
    }
}
