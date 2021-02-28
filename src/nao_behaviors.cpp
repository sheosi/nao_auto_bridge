#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/RunBehaviorAction.h>
#include <naoqi_bridge_msgs/GetInstalledBehaviors.h>


void SimulatedNao::OnRunBehavior() {
    //Note: We could replicate behaviour
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
        nao_connection.OnRunBehavior();
        naoqi_bridge_msgs::RunBehaviorResult result;
        act_srv_run_behavior->setSucceeded(result);
    }

    bool on_get_installed_behaviors(
        naoqi_bridge_msgs::GetInstalledBehaviors::Request &req,
        naoqi_bridge_msgs::GetInstalledBehaviors::Response &resp
    ) {
        nao_connection.OnGetInstalledBehaviors(resp);
        return true;
    }

    void init(ros::NodeHandle &n) {
        act_srv_run_behavior = std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction>>(new actionlib::SimpleActionServer<naoqi_bridge_msgs::RunBehaviorAction>(n, "run_behavior", on_run_behavior, true));
        srv_installed_behaviors = n.advertiseService("get_installed_behaviors", on_get_installed_behaviors);
    }
}   
