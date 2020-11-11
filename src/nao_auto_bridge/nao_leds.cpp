#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_msgs/BlinkGoal.h>
#include <naoqi_msgs/FadeRGB.h>

class SimulatedNao {
    void OnBlink(ros::SimpleActionServer &srv) {
        //Note: We could replicate behaviour
        naoqi_msgs::BlinkGoalResult res;
        srv.setSucceeded(res);
    }

    void OnFadeRgb(const naoqi_msgs::FadeRGBConstPtr &msg) {
    }
}

namespace BridgeNaoLeds {
    ros::SimpleActionServer act_srv_blink;
    ros::Subscriber sub_footstep;

    void init(ros::NodeHandle &n) {
        act_srv_blink = SimpleActionServer(n, "blink", on_blink, true);
        sub_footstep = n.subscribe("fade_rgb", 1000, on_fade_rgb);
    }

    void on_blink(
        const naoqi_msgs::BlinkGoalConstPtr &goal
    ) {
        std::get<SimulatedNao>(nao_connection).OnBlink(act_srv_blink);
    }

    void on_fade_rgb(
        const naoqi_msgs::FadeRGBConstPtr &msg
    ) {
        std::get<SimulatedNao>(nao_connection).OnFadeRgb(msg);
    }
}
