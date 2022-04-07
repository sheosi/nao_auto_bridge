#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <map>

namespace BridgeNaoDiagnosticUpdater {
    ros::Publisher pub_diagnostics;
    uint32 diag_upd_id;
    std::map<std::string, diagnostic_msgs::DiagnosticStatus> diagnostic_status_map;

    void init(ros::NodeHandle &n) {
        diag_upd_id = 0;
        diagnostic_status_map = std::map<std::string, diagnostic_msgs::DiagnosticStatus>();
        pub_diagnostics = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1000);
    }

    void republish_diagnostics() {
        diagnostic_msgs::DiagnosticArray msg;

        for(auto const& diag: diagnostic_status_map) {
            msg.status.push_back(imap.first);
        }

        msg.header.stamp = ros::Time::now();
        msg.header.seq = diag_upd_id;

        pub_diagnostics.publish(msg);

        diag_upd_id++;
    }

    void add_event(DiagnosticStatus &status) {
        diagnostic_status_map.insert({status.name, status});
        republish_diagnostics();
    }

    void remove_event(const std::string &name) {
        if (diagnostic_status_map.find(name) != diagnostic_status_map.end()) {
            diagnostic_status_map.erase(name);
            republish_diagnostics();
        }
    }
}
