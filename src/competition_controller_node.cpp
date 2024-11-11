#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ik_service/PoseIK.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <string>
#include <map>
#include <set>
#include <algorithm>

// Global vector to store orders 
std::vector<osrf_gear::Order> order_vector;

// Map to store the data of the logical camera 
std::map<std::string, osrf_gear::LogicalCameraImage> logical_camera_data;

// Buffer and Listener for tf2 transfromation 
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

// Sart the competition by calling the service client
bool startCompetition(ros::NodeHandle &nh) {
    ros::ServiceClient begin_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger begin_comp;

    ROS_INFO("Waiting for /ariac/start_competition service...");
    ros::service::waitForService("/ariac/start_competition");

    if (begin_client.call(begin_comp)) {
        if (begin_comp.response.success) {
            ROS_INFO("Competition started successfully: %s", begin_comp.response.message.c_str());
            return true;
        } else {
            ROS_WARN("Failed to start competition: %s", begin_comp.response.message.c_str());
            return false;
        }
    } else {
        ROS_ERROR("Failed to call /ariac/start_competition service.");
        return false;
    }
 
}

// Callback to subscribe to the order's topic 
void orderCallback(const osrf_gear::Order::ConstPtr& msg) {
    ROS_INFO("Recived order: %s", msg->order_id.c_str());
    order_vector.push_back(*msg); // Add the order
    ROS_INFO("Num of orders %lu", order_vector.size());
}

// Configure the subscriber of orders 
void receiveOrders(ros::NodeHandle &nh) {
    ros::Subscriber order_sub = nh.subscribe("/ariac/orders", 10, orderCallback);
    ROS_INFO("Subscribed to /ariac/orders");
    ros::spinOnce(); 
}
// Callback to proccess the information of the logic cameras 
void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, const std::string& camera_name) {
    if (!msg->models.empty()) {
        ROS_INFO("Camera %s detected %lu objects", camera_name.c_str(), msg->models.size());
        logical_camera_data[camera_name] = *msg; // Guardar la información de la cámara en el mapa
    }
}


// Funtion to subscribe to all camera
void subscribeToCameras(ros::NodeHandle &nh) {
    std::vector<std::string> camera_topics = {
        "/ariac/logical_camera_bin1",
        "/ariac/logical_camera_bin2",
        "/ariac/logical_camera_bin3",
        "/ariac/logical_camera_bin4",
        "/ariac/logical_camera_bin5",
        "/ariac/logical_camera_bin6",
        "/ariac/logical_camera_bin7",
        "/ariac/logical_camera_bin8",
        "/ariac/logical_camera_agv1",
        "/ariac/logical_camera_agv2"
    };

    for (const auto& topic : camera_topics) {
        ros::Subscriber sub = nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 10, 
            boost::bind(logicalCameraCallback, _1, topic));
        ROS_INFO("Subscribed to %s", topic.c_str());
    }
}

// Function to transform the position of the product 
bool transformPartPose(const std::string& camera_frame, const geometry_msgs::Pose& part_pose, geometry_msgs::PoseStamped& goal_pose) {
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.frame_id = camera_frame;
    part_pose_stamped.pose = part_pose;

    try {
        geometry_msgs::TransformStamped tfStamped;
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame, ros::Time(0.0), ros::Duration(1.0));
        tf2::doTransform(part_pose_stamped, goal_pose, tfStamped);
        ROS_INFO("Transformed pose to arm coordinates: (%.2f, %.2f, %.2f)", 
            goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return false;
    }
}


// Process the orders
void processOrders(ros::NodeHandle &nh) {
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("/compute_ik");
    ik_service::PoseIK ik_srv;
    
    while (ros::ok()) {
        if (!order_vector.empty()) {
            osrf_gear::Order current_order = order_vector.front();
            if (!current_order.shipments.empty() && !current_order.shipments[0].products.empty()) {
                osrf_gear::Product product = current_order.shipments[0].products[0];

                // Obtener el nombre del bin de la cámara lógica
                std::string bin_name = "logical_camera_bin4_frame";
                if (logical_camera_data.find(bin_name) != logical_camera_data.end()) {
                    geometry_msgs::Pose part_pose = logical_camera_data[bin_name].models[0].pose;
                    geometry_msgs::PoseStamped goal_pose;
                    
                    // Transformar la pose al marco del brazo
                    if (transformPartPose(bin_name, part_pose, goal_pose)) {
                        goal_pose.pose.position.z += 0.10; // Añadir 10 cm en z para evitar colisiones
                        ik_srv.request.pose = goal_pose.pose;
                        
                        // Llamar al servicio IK
                        if (ik_client.call(ik_srv) && ik_srv.response.success) {
                            ROS_INFO("IK solution found and arm moved successfully.");
                        } else {
                            ROS_ERROR("Failed to move arm using IK service.");
                        }
                    }
                }
            }
            order_vector.erase(order_vector.begin());
        }

        ros::spinOnce();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "competition_controller_node");
    ros::NodeHandle nh;
    tfListener(tfBuffer);

    if (!startCompetition(nh)) return 1;

    // Configure the subscriber to recive orders 
    receiveOrders(nh);
    subscribeToCameras(nh);
    // Process the orders 
    processOrders(nh);
    return 0;
}