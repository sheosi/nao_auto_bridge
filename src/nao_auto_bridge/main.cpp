#include "nao_auto_bridge.h"
#include <ros/ros.h>

std::variant <NetworkNao,SimulatedNao> nao_connection = SimulatedNao();

int main(int argc, char** argv) {
    ros::init(argc, argv, "nao_auto_bridge");

    ros::NodeHandle n;
    BridgeNaoAlife::init(n);
    BridgeNaoBehaviors::init(n);
    BridgeNaoDiagnosticUpdater::init(n);
    BridgeNaoFootsteps::init(n);
    BridgeNaoLeds::init(n);
    BridgeNaoSpeech::init(n);
    BridgeNaoTactile::init(n);
    BridgeNaoWalker::init(n);

    ros::spin();

    return 0;
}
