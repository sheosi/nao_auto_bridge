#include "nao_auto_bridge.h"
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <humanoid_nav_msgs/ClipFootstep.h>

void SimulatedNao::OnFootstep(const humanoid_nav_msgs::StepTarget& msg) {
    //Note: We could replicate behaviour
    this->OnCmdStep(msg);
}

humanoid_nav_msgs::StepTarget SimulatedNao::OnClipFootstep(const humanoid_nav_msgs::StepTarget& in) {
    humanoid_nav_msgs::StepTarget result = in;
    return result;
}

namespace BridgeNaoFootsteps {
    ros::Subscriber sub_footstep;
    ros::ServiceServer srv_footstep_srv,
                       srv_clip_footstep;

    void on_footstep(const humanoid_nav_msgs::StepTarget::ConstPtr& msg) {
        nao_connection.OnFootstep(*msg);
    }

    bool on_footstep_srv(
        humanoid_nav_msgs::StepTargetService::Request &req,
        humanoid_nav_msgs::StepTargetService::Response &resp
    ) {
        nao_connection.OnFootstep(req.step);
        
        return true;
    }

    bool on_clip_footstep_srv(
        humanoid_nav_msgs::ClipFootstep::Request &req,
        humanoid_nav_msgs::ClipFootstep::Response &resp
    ) {
        resp.step = nao_connection.OnClipFootstep(req.step);
        return true;
    }

        void init(ros::NodeHandle &n){
        float init_stiffness; //Note: We could replicate behaviour
        n.param("init_stiffness", init_stiffness, 0.0f);

        sub_footstep = n.subscribe("footstep", 1000, on_footstep);

        srv_footstep_srv = n.advertiseService("footstep_srv", on_footstep_srv);
        srv_clip_footstep = n.advertiseService("clip_footstep_srv", on_clip_footstep_srv);

    }
}
