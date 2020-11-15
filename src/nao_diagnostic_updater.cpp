#include "nao_auto_bridge.h"
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace BridgeNaoDiagnosticUpdater {
    ros::Publisher pub_diagnostics;
    void init(ros::NodeHandle &n) {
        pub_diagnostics = n.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1000);
    }
}
