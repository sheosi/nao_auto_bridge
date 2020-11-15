#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <std_msgs/Bool.h>


namespace BridgeNaoTactile {
    ros::Publisher pub_tactile_touch,
                   pub_bumper,
                   pub_foot_contact;

    void init(ros::NodeHandle &n) {
        pub_tactile_touch = n.advertise<naoqi_bridge_msgs::HeadTouch>("tactile_touch", 1000);
        pub_bumper = n.advertise<naoqi_bridge_msgs::Bumper>("bumper", 1000);
        pub_foot_contact = n.advertise<std_msgs::Bool>("foot_contact", 1000);
    }
}
