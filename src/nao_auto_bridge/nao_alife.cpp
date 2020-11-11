#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Note: Some methods are only implemented for a simulated Nao

class NetworkNao {
};

class SimulatedNao {
    void OnDisabled() {
        //Note: We could replicate behaviour
    }

    void OnInteractive() {
        //Note: We could replicate behaviour
    }

    void OnSafeguard() {
        //Note: We could replicate behaviour
    }

    void OnSolitary() {
        //Note: We could replicate behaviour
    }
};

namespace BridgeNaoAlife {
    ros::ServiceServer srv_disabled,
                       srv_interactive,
                       srv_safeguard,
                       srv_solitary;

    bool init(ros::NodeHandle &n) {
        if std::holds_alternative<NetworkNao> {
            srv_disabled = n.advertiseService("disabled", BridgeNaoAlife::on_disabled);
            srv_interactive = n.advertiseService("interactive", BridgeNaoAlife::on_interactive);
            srv_safeguard =  n.advertiseService("safeguard", BridgeNaoAlife::on_safeguard);
            srv_solitary =  n.advertiseService("solitary", BridgeNaoAlife::on_solitary);
        }
    }

    bool on_disabled(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ) {

        std::get<SimulatedNao>(nao_connection).OnDisabled();
        return true;
    }

    bool on_interactive(
            std_srvs::Empty::Request &req,
            std_srvs::Empty::Response &resp
    ) {

        std::get<SimulatedNao>(nao_connection).OnInteractive();
        return true;
    }

    bool on_safeguard(
                std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &resp
    ) {

        std::get<SimulatedNao>(nao_connection).OnSafeguard();
        return true;
    }

    bool on_solitary(
                std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &resp
    ) {

        std::get<SimulatedNao>(nao_connection).OnSolitary();
        return true;
    }
}


