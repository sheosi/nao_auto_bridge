#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/BlinkGoal.h>
#include <naoqi_bridge_msgs/FadeRGB.h>

void SimulatedNao::OnBlink() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnFadeRgb(const naoqi_bridge_msgs::FadeRGBConstPtr &msg) {
}

namespace BridgeNaoLeds {
    std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>> act_srv_blink;
    ros::Subscriber sub_footstep;

    void on_blink(
        const naoqi_bridge_msgs::BlinkGoalConstPtr &goal
    ) {
        boost::get<SimulatedNao>(nao_connection).OnBlink();
        naoqi_bridge_msgs::BlinkResult res;
        act_srv_blink->setSucceeded(res);
    }

    void on_fade_rgb(
        const naoqi_bridge_msgs::FadeRGBConstPtr &msg
    ) {
        boost::get<SimulatedNao>(nao_connection).OnFadeRgb(msg);
    }

    void init(ros::NodeHandle &n) {
        act_srv_blink = std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>>(new actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>(n, "blink", on_blink, true));
        sub_footstep = n.subscribe("fade_rgb", 1000, on_fade_rgb);
    }
}
