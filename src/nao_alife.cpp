#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// SimulatedNao
void  SimulatedNao::OnDisabled() {
    //Note: We could replicate behaviour
    ROS_INFO("Set Nao autonomous behaviour as 'Disabled'");
}

void SimulatedNao::OnInteractive() {
    //Note: We could replicate behaviour
    ROS_INFO("Set Nao autonomous behaviour as 'Interactive'");
}

void SimulatedNao::OnSafeguard() {
    //Note: We could replicate behaviour
    ROS_INFO("Set Nao autonomous behaviour as 'SafeGuard'");
}

void SimulatedNao::OnSolitary() {
    //Note: We could replicate behaviour
    ROS_INFO("Set Nao autonomous behaviour as 'Solitary'");
}


namespace BridgeNaoAlife {
    ros::ServiceServer srv_disabled,
                       srv_interactive,
                       srv_safeguard,
                       srv_solitary;

    bool on_disabled(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {

        boost::get<SimulatedNao>(nao_connection).OnDisabled();
        return true;
    }

    bool on_interactive(
            std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp
    ) {

        boost::get<SimulatedNao>(nao_connection).OnInteractive();
        return true;
    }

    bool on_safeguard(
                std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &resp
    ) {

        boost::get<SimulatedNao>(nao_connection).OnSafeguard();
        return true;
    }

    bool on_solitary(
                std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &resp
    ) {

        boost::get<SimulatedNao>(nao_connection).OnSolitary();
        return true;
    }

    void init(ros::NodeHandle &n) {
        if (holds_alternative<NetworkNao>(nao_connection)) {
            srv_disabled = n.advertiseService("nao_alife/disabled", BridgeNaoAlife::on_disabled);
            srv_interactive = n.advertiseService("nao_alife/interactive", BridgeNaoAlife::on_interactive);
            srv_safeguard =  n.advertiseService("nao_alife/safeguard", BridgeNaoAlife::on_safeguard);
            srv_solitary =  n.advertiseService("nao_alife/solitary", BridgeNaoAlife::on_solitary);
        }
    }
}



