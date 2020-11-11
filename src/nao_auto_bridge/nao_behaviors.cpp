#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_msgs/RunBehaviorGoal.h>
#include <naoqi_msgs/GetInstalledBehaviors.h>

class SimulatedNao {
    void OnRunBehavior(ros::SimpleActionServer &srv) {
        //Note: We could replicate behaviour
        naoqi_msgs::RunBehaviorGoalResult res;
        srv.setSucceeded(res);
    }

    void OnGetInstalledBehaviors(naoqi_msgs::GetInstalledBehaviors::Response &resp) {
        //Note: We could replicate behaviour
    }
}

namespace BridgeNaoBehaviors {
    ros::SimpleActionServer act_srv_run_behavior;
    ros::ServiceServer srv_installed_behaviors;

    void init(ros::NodeHandle &n) {
        act_srv_run_behavior = SimpleActionServer(n, "run_behavior", on_run_behavior, true);
        srv_installed_behaviors = n.advertiseServer("get_installed_behaviors", on_get_installed_behaviors);
    }

    void on_run_behavior(
            const naoqi_msgs::GetInstalledBehaviorsConstPtr &goal
        ) {
        std::get<SimulatedNao>(nao_connection).OnRunBehavior(act_srv_run_behavior);
    }

    bool on_get_installed_behaviors(
        naoqi_msgs::GetInstalledBehaviors::Request &req,
        naoqi_msgs::GetInstalledBehaviors::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnGetInstalledBehaviors(resp);
        return true;
    }
}   
