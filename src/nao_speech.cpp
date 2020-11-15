#include "nao_auto_bridge.h"
#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackAction.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyAction.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>


void SimulatedNao::OnSpeechActionGoal() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnSpeechVocabularyAction() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnSpeech() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnReconfigure() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnStartRecognition() {
    //Note: We could replicate behaviour
}

void SimulatedNao::OnStopRecognition() {
    //Note: We could replicate behaviour
}



namespace BridgeNaoSpeech {
    std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SpeechWithFeedbackAction>> act_srv_speech_action_goal;
    std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SetSpeechVocabularyAction>> act_srv_speech_vocabulary_action;

    ros::Subscriber sub_speech;
    ros::ServiceServer srv_reconfigure,
                       srv_start_recognition,
                       srv_stop_recognition;

    void on_speech_action_goal(
        const naoqi_bridge_msgs::SpeechWithFeedbackGoalConstPtr &goal
    ) {
        boost::get<SimulatedNao>(nao_connection).OnSpeechActionGoal();
    }

    void on_speech_vocabulary_action(
        const naoqi_bridge_msgs::SetSpeechVocabularyGoalConstPtr &goal
    ) {
        boost::get<SimulatedNao>(nao_connection).OnSpeechVocabularyAction();
    }

    void on_speech(
        const std_msgs::StringConstPtr &msg
    ) {
        boost::get<SimulatedNao>(nao_connection).OnSpeech();
    }

    bool on_reconfigure(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        boost::get<SimulatedNao>(nao_connection).OnReconfigure();
        return true;
    }

    bool on_start_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        boost::get<SimulatedNao>(nao_connection).OnStartRecognition();
        return true;
    }

    bool on_stop_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        boost::get<SimulatedNao>(nao_connection).OnStopRecognition();
        return true;
    }

    void init(ros::NodeHandle &n) {
        std::string voice, language;
        float volume;
        std::vector<std::string> vocabulary;
        bool enable_audio_expression, enable_visual_expression, word_spoting;

        n.param("voice", voice, std::string(""));
        n.param("language", language, std::string(""));
        n.param("volume", volume, 0.0f);
        n.param("vocabulary", vocabulary, std::vector<std::string>());
        n.param("enable_audio_expression", enable_audio_expression, false);
        n.param("enable_visual_expression", enable_visual_expression, false);
        n.param("word_spoting", word_spoting, false);

        act_srv_speech_action_goal = 
        std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SpeechWithFeedbackAction>>(new actionlib::SimpleActionServer<naoqi_bridge_msgs::SpeechWithFeedbackAction>(n, "speech_action/goal", on_speech_action_goal, true));
        act_srv_speech_vocabulary_action = 
        std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SetSpeechVocabularyAction>>(
        new actionlib::SimpleActionServer<naoqi_bridge_msgs::SetSpeechVocabularyAction>(n, "/speech_vocabulary_action/goal", on_speech_vocabulary_action, true));

        sub_speech = n.subscribe("speech", 1000, on_speech);

        srv_reconfigure = n.advertiseService("reconfigure", on_reconfigure);
        srv_start_recognition = n.advertiseService("start_recognition", on_start_recognition);
        srv_stop_recognition = n.advertiseService("stop_recognition", on_stop_recognition);
    }
}
