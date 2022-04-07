#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/BlinkGoal.h>
#include <naoqi_bridge_msgs/FadeRGB.h>

std::string rgba_to_str(const std_msgs::ColorRGBA &rgba) {
    std::stringstream rgba;
    rgba << "RGBA(" << 
        color.r << ", " << 
        color.g << ", " <<
        color.b << ", " <<
        color.a << ")";
    
    rgba.str();
}

float duration_to_secs(const std_msgs::Duration &duration) {
    return duration.secs + duration.nsecs / 1e9;
}

void SimulatedNao::OnBlink(const naoqi_bridge_msgs::BlinkGoalConstPtr &goal) {
    //Note: We could replicate behaviour
    std::stringstream colors;
    for(auto color : goal.colors) {
        colors << rgba_to_str(color) << ", ";
    }
    
    ROS_INFO("Nao blinked with colors %s, background %f secs with a rate of %f Hz and a deviation of %f", 
        colors.str().c_str(),
        rgba_to_str(goal.background).c_str(),
        duration_to_secs(goal.blink_duration),
        goal.blink_rate_mean,
        goal.blink_rate_sd
    );
}

void SimulatedNao::OnFadeRgb(const naoqi_bridge_msgs::FadeRGBConstPtr &msg) {
    ROS_INFO("Led '%s' faded to color %s for %f seconds",
        msg.led_name.c_str(),
        rgba_to_str(msg.color).c_str(),
        duration_to_secs(msg.fade_duration)
    );
}

namespace BridgeNaoLeds {
    std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>> act_srv_blink;
    ros::Subscriber sub_footstep;

    void on_blink(
        const naoqi_bridge_msgs::BlinkGoalConstPtr &goal
    ) {
        nao_connection.OnBlink();
        naoqi_bridge_msgs::BlinkResult res;
        act_srv_blink->setSucceeded(res);
    }

    void on_fade_rgb(
        const naoqi_bridge_msgs::FadeRGBConstPtr &msg
    ) {
        nao_connection.OnFadeRgb(msg);
    }

    void init(ros::NodeHandle &n) {
        act_srv_blink = std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>>(new actionlib::SimpleActionServer<naoqi_bridge_msgs::BlinkAction>(n, "blink", on_blink, true));
        sub_footstep = n.subscribe("fade_rgb", 1000, on_fade_rgb);
    }
}
