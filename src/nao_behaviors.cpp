#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/RunBehaviorAction.h>
#include <naoqi_bridge_msgs/GetInstalledBehaviors.h>


void SimulatedNao::OnRunBehavior(actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction> &srv) {
    //Note: We could replicate behaviour
    naoqi_bridge_msgs::RunBehaviorResult res;
    srv.setSucceeded(res);
}

void SimulatedNao::OnGetInstalledBehaviors(naoqi_bridge_msgs::GetInstalledBehaviors::Response &resp) {
    //Note: We could replicate behaviour
}

namespace BridgeNaoBehaviors {
    std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction>> act_srv_run_behavior;
    ros::ServiceServer srv_installed_behaviors;

    void on_run_behavior(
            const naoqi_bridge_msgs::RunBehaviorGoalConstPtr &goal
        ) {
        boost::get<SimulatedNao>(nao_connection).OnRunBehavior(*act_srv_run_behavior);
    }

    bool on_get_installed_behaviors(
        naoqi_bridge_msgs::GetInstalledBehaviors::Request &req,
        naoqi_bridge_msgs::GetInstalledBehaviors::Response &resp
    ) {
        boost::get<SimulatedNao>(nao_connection).OnGetInstalledBehaviors(resp);
        return true;
    }

    void init(ros::NodeHandle &n) {
        act_srv_run_behavior = std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction>>(new actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction>(n, "run_behavior", on_run_behavior, true));
        srv_installed_behaviors = n.advertiseService("get_installed_behaviors", on_get_installed_behaviors);
    }
}   
