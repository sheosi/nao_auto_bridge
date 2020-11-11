#include <boost/variant.hpp>
#include <ros/ros.h>

// Some forward definitions
namespace naoqi_msgs {
    namespace GetInstalledBehaviors {
        class Response;
    }
}

// Base classes
class NetworkNao {
private:
    ros::NodeHandle n;

public:
    NetworkNao() {}

    // No Alife methods

};

class SimulatedNao {
public:
    // Alife methods
    void OnDisabled();
    void OnInteractive();
    void OnSafeguard();
    void OnSolitary();

    // Behaviors methods
    void OnRunBehavior(ros::SimpleActionServer &srv);
    void OnGetInstalledBehaviors(naoqi_msgs::GetInstalledBehaviors::Response &resp);

    // Diagnostic methods
    // No methods

    // Footsteps methods
    void OnFootstep(const humanoid_nav_msgs::StepTarget::ConstPtr& msg);
    humanoid_nav_msgs::StepTarget OnClipFootstep(const humanoid_nav_msgs::StepTarget::ConstPtr& in);

    // Leds methods
    void OnBlink(ros::SimpleActionServer &srv);
    humanoid_nav_msgs::StepTarget OnClipFootstep(
        const humanoid_nav_msgs::StepTarget::ConstPtr& in
    );

    // Speech methods
    void OnSpeechActionGoal();
    void OnSpeechVocabularyAction();
    void OnSpeech();
    void OnReconfigure();
    void OnStartRecognition();
    void OnStopRecognition();

    // Walker methods
    void OnCmdVel();
    void OnCmdPose();
    void OnCmdStep();
    void OnReadFootGaitConfigSrv();
    void OnCmdVelSrv();
    void OnCmdStepSrv();
    void OnCmdPoseSrv();
    void OnStopWalkSrv();
    void OnNeedsStartWalkPoseSrv();
    void OnEnableArmWalkingSrv();
};

// Bridges
namespace BridgeNaoAlife {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoBehaviors {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoDiagnosticUpdater {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoFootsteps {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoLeds {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoSpeech {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoTactile {
    void init(ros::NodeHandle &n);
}

namespace BridgeNaoWalker {
    void init(ros::NodeHandle &n);
}

extern boost::variant <NetworkNao,SimulatedNao> nao_connection;
