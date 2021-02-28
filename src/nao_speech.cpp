#include "nao_auto_bridge.h"

#include <boost/algorithm/string/join.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackAction.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyAction.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

using std::string;

void SimulatedNao::OnSpeechActionGoal(const string& text) {
    //Note: We could replicate behaviour
    ROS_INFO("Nao said: %s", text.c_str());
}

void SimulatedNao::OnSpeechVocabularyAction(const std::vector<string>& words) {
    //Note: We could replicate behaviour
    const string joined = boost::algorithm::join(words, ",");
    ROS_INFO("Words recognized set: %s", joined.c_str());
}

void SimulatedNao::OnSpeech(const string& text) {
    //Note: We could replicate behaviour
    ROS_INFO("Nao said: %s", text.c_str());
}

void SimulatedNao::OnReconfigure() {
    //Note: We could replicate behaviour
    ROS_INFO("Speech reconfigured");
}

void SimulatedNao::OnStartRecognition() {
    //Note: We could replicate behaviour
    if (!this->speech_data.recognition_started) {
        ROS_INFO("Recognition started");
        this->speech_data.recognition_started = true;
    }
    else {
        ROS_INFO("Tried to start recognition, but it was already started by another node");
    }
}

void SimulatedNao::OnStopRecognition() {
    //Note: We could replicate behaviour
    if (this->speech_data.recognition_started) {
        ROS_INFO("Recognition stopped");
        this->speech_data.recognition_started = false;
    }
    else {
        ROS_INFO("Tried top stop recognition, but it is already stopped");
    }
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
        nao_connection.OnSpeechActionGoal(goal->say);
        naoqi_bridge_msgs::SpeechWithFeedbackResult result;
        act_srv_speech_action_goal->setSucceeded(result);
    }

    void on_speech_vocabulary_action(
        const naoqi_bridge_msgs::SetSpeechVocabularyGoalConstPtr &goal
    ) {
        nao_connection.OnSpeechVocabularyAction(goal->words);
        naoqi_bridge_msgs::SetSpeechVocabularyResult result;
        act_srv_speech_vocabulary_action->setSucceeded(result);
    }

    void on_speech(
        const std_msgs::StringConstPtr &msg
    ) {
        nao_connection.OnSpeech(msg->data);
    }

    bool on_reconfigure(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        nao_connection.OnReconfigure();
        return true;
    }

    bool on_start_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        nao_connection.OnStartRecognition();
        return true;
    }

    bool on_stop_recognition(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &resp
    ){
        nao_connection.OnStopRecognition();
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
        std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SpeechWithFeedbackAction>>(
            new actionlib::SimpleActionServer<naoqi_bridge_msgs::SpeechWithFeedbackAction>(n, "speech_action/goal", on_speech_action_goal, true));
        act_srv_speech_vocabulary_action = 
        std::unique_ptr<actionlib::SimpleActionServer<naoqi_bridge_msgs::SetSpeechVocabularyAction>>(
            new actionlib::SimpleActionServer<naoqi_bridge_msgs::SetSpeechVocabularyAction>(n, "/speech_vocabulary_action/goal", on_speech_vocabulary_action, true));

        sub_speech = n.subscribe("speech", 1000, on_speech);

        srv_reconfigure = n.advertiseService("reconfigure", on_reconfigure);
        srv_start_recognition = n.advertiseService("start_recognition", on_start_recognition);
        srv_stop_recognition = n.advertiseService("stop_recognition", on_stop_recognition);
    }
}
