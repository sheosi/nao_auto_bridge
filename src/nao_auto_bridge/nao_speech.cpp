#include "nao_auto_bridge.h"
#include <vector>
#include <ros/ros.h>
#include <actionlib/server/SimpleActionServer.h>
#include <naoqi_msgs/SpeechWithFeedback.h>
#include <naoqi_msgs/SetSpeechVocabulary.h>
#include <naoqi_msgs/WordRecognized.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


class SimulatedNao {
    void OnSpeechActionGoal() {
        //Note: We could replicate behaviour
    }

    void OnSpeechVocabularyAction() {
        //Note: We could replicate behaviour
    }

    void OnSpeech() {
        //Note: We could replicate behaviour
    }

    void OnReconfigure() {
        //Note: We could replicate behaviour
    }

    void OnStartRecognition() {
        //Note: We could replicate behaviour
    }

    void OnStopRecognition() {
        //Note: We could replicate behaviour
    }

}

namespace BridgeNaoSpeech {
    ros::SimpleActionServer act_srv_speech_action_goal,
                            act_srv_speech_vocabulary_action;
    ros::Subscriber sub_speech;
    ros::ServiceServer srv_reconfigure,
                       srv_start_recognition,
                       srv_stop_recognition;

    void init(ros::NodeHandle &n) {
        std::string voice, language;
        float volume;
        std::vector<String> vocabulary;
        bool enable_audio_expression, enable_visual_expression, word_spoting;

        n.param("voice", voice, "");
        n.param("language", language, "");
        n.param("volume", volume, "");
        n.param("vocabulary", vocabulary, []);
        n.param("enable_audio_expression", enable_audio_expression, false);
        n.param("enable_visual_expression", enable_visual_expression, false);
        n.param("word_spoting", word_spoting, false);

        act_srv_speech_action_goal = SimpleActionServer(n, "speech_action/goal", on_speech_action_goal, true);
        act_srv_speech_vocabulary_action = SimpleActionServer(n, "/speech_vocabulary_action/goal", on_speech_vocabulary_action, true);

        sub_speech = n.subscribe("speech", 1000, on_speech);

        srv_reconfigure = n.advertiseService("reconfigure", on_reconfigure);
        srv_start_recognition = n.advertiseService("start_recognition", on_start_recognition);
        srv_stop_recognition = n.advertiseService("stop_recognition", on_stop_recognition);
    }

    void on_speech_action_goal(
        const naoqi_msgs::SpeechWithFeedbackConstPtr &goal
    ) {
        std::get<SimulatedNao>(nao_connection).OnSpeechActionGoal();
    }

    void on_speech_vocabulary_action(
        const naoqi_msgs::SetSpeechVocabularyConstPtr &goal
    ) {
        std::get<SimulatedNao>(nao_connection).OnSpeechVocabularyAction();
    }

    void on_speech(
        const std_msgs::StringConstPtr &msg
    ) {
        std::get<SimulatedNao>(nao_connection).OnSpeech();
    }

    void on_reconfigure(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        std::get<SimulatedNao>(nao_connection).OnReconfigure();
    }

    void on_start_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        std::get<SimulatedNao>(nao_connection).OnStartRecognition();
    }

    void on_stop_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        std::get<SimulatedNao>(nao_connection).OnStopRecogntion();
    }
}
