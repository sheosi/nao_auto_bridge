#include <boost/variant.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <humanoid_nav_msgs/ClipFootstep.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <humanoid_nav_msgs/StepTargetService.h>
#include <naoqi_bridge_msgs/GetInstalledBehaviors.h>
#include <naoqi_bridge_msgs/FadeRGB.h>
#include <naoqi_bridge_msgs/RunBehaviorAction.h>
#include <naoqi_bridge_msgs/BlinkAction.h>

using std::string;

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
    void OnRunBehavior();
    void OnGetInstalledBehaviors(naoqi_bridge_msgs::GetInstalledBehaviors::Response &resp);

    // Diagnostic methods
    // No methods

    // Footsteps methods
    void OnFootstep(const humanoid_nav_msgs::StepTarget& msg);
    humanoid_nav_msgs::StepTarget OnClipFootstep(const humanoid_nav_msgs::StepTarget& in);

    // Leds methods
    void OnBlink();
    void OnFadeRgb(const naoqi_bridge_msgs::FadeRGBConstPtr &msg);

    // Speech methods
    void OnSpeechActionGoal(const string& text);
    void OnSpeechVocabularyAction(const std::vector<string>& words);
    void OnSpeech(const string& text);
    void OnReconfigure();
    void OnStartRecognition();
    void OnStopRecognition();

    // Walker methods
    void OnCmdVel(const geometry_msgs::Twist &msg);
    void OnCmdPose(const geometry_msgs::Pose2D &msg);
    void OnCmdStep(const humanoid_nav_msgs::StepTarget &msg);
    void OnReadFootGaitConfigSrv();
    void OnCmdVelSrv(const geometry_msgs::Twist &msg);
    void OnCmdStepSrv(const humanoid_nav_msgs::StepTarget &msg);
    void OnCmdPoseSrv(const geometry_msgs::Pose2D &msg);
    void OnStopWalkSrv();
    void OnNeedsStartWalkPoseSrv();
    void OnEnableArmWalkingSrv(bool left, bool right);

private:
    struct SpeechData {
        bool recognition_started;
        bool words_set;

        SpeechData() : recognition_started(false), words_set(false){
        }
    };
    SpeechData speech_data;
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

template <typename T, typename... Ts>
bool holds_alternative(const boost::variant<Ts...>& v) noexcept
{
    return boost::get<T>(&v) != nullptr;
}