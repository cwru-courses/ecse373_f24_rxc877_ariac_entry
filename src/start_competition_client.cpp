#include "ros/ros.h"
#include "std_srvs/Trigger.h"

int main(int argc, char **argv) {
    // Iiniziate the node
    ros::init(argc, argv, "start_competition_node");
    ros::NodeHandle n;

    // Create a client for the service '/ariac/start_competition'
    ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    // CReate a trigger object
    std_srvs::Trigger srv;

    if(!client.waitForExistence(ros::Duration(5.0))){
        ROS_ERROR("The service /ariac/start_competition is not avaible");
        return 1;
    }

    // Call service
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Competición iniciada con éxito: %s", srv.response.message.c_str());
        } else {
            ROS_WARN("No se pudo iniciar la competición: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Error al llamar al servicio /ariac/start_competition");
        return 1;
    }

    ros::spinOnce();
    
    return 0;
}
