#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <humanoid_nav_msgs/StepTarget.h>

class SimulatedNao {
    void OnFootstep(const humanoid_nav_msgs::StepTarget::ConstPtr& msg) {
        //Note: We could replicate behaviour
    }

    humanoid_nav_msgs::StepTarget OnClipFootstep(
        const humanoid_nav_msgs::StepTarget::ConstPtr& in
    ) {
        //Note: We could replicate behaviour
        humanoid_nav_msgs::StepTarget result = in;
        return result;
    }
}

namespace BridgeNaoFootsteps {
    ros::Subscriber sub_footstep;
    ros::ServiceServer srv_footstep_srv,
                       srv_clip_footstep;

    void init(ros::NodeHandle &n){
        float init_stiffness; //Note: We could replicate behaviour
        n.param("init_stiffness", init_stiffness, 0);

        sub_footstep = n.subscribe("footstep", 1000, on_footstep);

        srv_footstep_srv = n.advertiseService("footstep_srv", on_footstep_srv);
        srv_clip_footstep = n.advertiseService("clip_footstep_srv", on_clip_footstep_srv);

    }

    void on_footstep(const humanoid_nav_msgs::StepTarget::ConstPtr& msg) {
        std::get<SimulatedNao>(nao_connection).OnFootstep(msg);
    }

    bool on_footstep_srv(
        humanoid_nav_msgs::StepTargetService::Request &req,
        humanoid_nav_msgs::StepTargetService::Response &resp
    ) {
        std::get<SimulatedNao>(nao_connection).OnFootstep(req.step);
        return true;
    }

    bool on_clip_footstep_srv(
        humanoid_nav_msgs::ClipFootstep::Request &req,
        humanoid_nav_msgs::ClipFootstep::Response &resp
    ) {
        const auto res = std::get<SimulatedNao>(nao_connection).OnClipFootstep(req.step);
        resp.step = res;
        return true;
    }
}
