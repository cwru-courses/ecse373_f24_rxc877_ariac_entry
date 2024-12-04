#include "ros/ros.h"

#include "std_srvs/Trigger.h"

#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"

#include "ik_service/PoseIK.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <vector>
#include <string>
#include <map>
#include <set>
#include <algorithm>
#include <unordered_map>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <actionlib/client/simple_action_client.h>

#include "sensor_msgs/JointState.h"

// Action Server headers
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
// The Action Server "message type"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/JointTrajectoryFeedback.h"
#include "control_msgs/FollowJointTrajectoryResult.h"

#include <std_msgs/Header.h>

// Instantiate the Action Server client

// The header and goal (not the tolerances) of the action must be filled.


struct DetectedPart {
    geometry_msgs::Pose part_pose;    // Pose of the detected part
    std::string source_frame_id;      // Source frame of the detected part
    bool has_been_picked = false;     // Tracks if the part has been picked
    bool marked_as_obsolete = false; // Flags parts that are no longer detected
};
// Global variable to store th e current state of the joints 
sensor_msgs::JointState current_joint_states;
std::vector<std::string> joint_names;


ros::Time last_joint_state_time;
// Global vector to store orders 
std::vector<osrf_gear::Order> order_vector;
std::map<std::string, std::vector<DetectedPart>> product_locations;
//ros::Publisher trajectory_pub;


// Map to store the data of the logical camera 
std::map<std::string, osrf_gear::LogicalCameraImage> logical_camera_data;

// Buffer and Listener for tf2 transfromation 
// tf2_ros::Buffer tfBuffer;
// tf2_ros::TransformListener tfListener(tfBuffer);

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac("ariac/arm/follow_joint_trajectory", true);

 std::mutex data_mutex; 



void goalActiveCallback() {
    ROS_INFO("Goal has been activated!");
}

void feedbackCallback(const boost::shared_ptr<const control_msgs::FollowJointTrajectoryFeedback>& feedback) {
    ROS_INFO_STREAM("Current arm state: " ); // desirde actual or error
}

void resultCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& res) {
  ROS_INFO("Action complete.");
    }



// Sart the competition by calling the service client
bool startCompetition(ros::NodeHandle &nh) {
    ros::ServiceClient begin_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger begin_comp;

    ROS_INFO("Waiting for /ariac/start_competition service...");
     int retries = 0;
    const int max_retries = 10;  // Número máximo de intentos
    while (!begin_client.exists() && retries < max_retries) {
        ROS_INFO("Service not available yet. Retrying... (%d/%d)", retries + 1, max_retries);
        ros::Duration(2.0).sleep();  // Espera 2 segundos antes de volver a intentar
        retries++;
    }

    if (retries == max_retries) {
        ROS_ERROR("Failed to find /ariac/start_competition service after multiple attempts.");
        return false;
    }

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
   // ros::spinOnce(); 
}

// Callback to proccess the information of the logic cameras 
void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, const std::string& camera_name) {
    if (!msg->models.empty()) {
        ROS_INFO("Camera %s detected %lu objects", camera_name.c_str(), msg->models.size());
        logical_camera_data[camera_name] = *msg; // Guardar la información de la cámara en el mapa
    }
}

// Function to transform the position of the product 
bool transformPartPose(const std::string& camera_frame, tf2_ros::Buffer& tfBuffer,
                       const geometry_msgs::Pose& part_pose, 
                       geometry_msgs::PoseStamped& goal_pose) {
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.frame_id = camera_frame;
    part_pose_stamped.header.stamp = ros::Time(0); // Usar el último frame disponible
    part_pose_stamped.pose = part_pose;

    try {
        // Intentar la transformación al marco del brazo
        geometry_msgs::TransformStamped tfStamped;
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", camera_frame, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(part_pose_stamped, goal_pose, tfStamped);
        ROS_INFO("Pose detectada por la cámara (marco %s): (%.2f, %.2f, %.2f)", 
                camera_frame.c_str(), part_pose.position.x, part_pose.position.y, part_pose.position.z);

        ROS_INFO("Pose transformada al marco 'arm1_base_link': (%.2f, %.2f, %.2f)", 
                 goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Error al transformar la pose: %s", ex.what());
        return false;
    }
}


// Funtion to subscribe to all camera
void subscribeToCameras(ros::NodeHandle &nh) {
    std::vector<std::string> camera_topics = {
        "/ariac/bin1",
        "/ariac/bin2",
        "/ariac/bin3",
        "/ariac/bin4",
        "/ariac/bin5",
        "/ariac/bin6",
        "/ariac/bin7",
        "/ariac/bin8",
        "/ariac/agv1",
        "/ariac/agv2"
    };

    for (const auto& topic : camera_topics) {
        ros::Subscriber sub = nh.subscribe<osrf_gear::LogicalCameraImage>(topic, 10, 
            boost::bind(logicalCameraCallback, _1, topic));
        ROS_INFO("Subscribed to %s", topic.c_str());
    }
}


bool getJointNames(std::vector<std::string>& joint_names) {
    // Obtener los nombres de las juntas desde el servidor de parámetros
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("Failed to retrieve joint names from the parameter server.");
        return false;
    }
    ROS_INFO("Retrieved joint names:");
    for (const auto &joint : joint_names) {
        ROS_INFO(" - %s", joint.c_str());
    }
    if (joint_names.size() != 7) {
        ROS_ERROR("Expected 7 joint names, but got %lu.", joint_names.size());
        return false; // Return false if validation fails
    }
    return true;
}


bool waitForJointStates(const std::vector<std::string>& joint_names, int timeout_sec = 10) {
    ROS_INFO("Waiting for joint_states to be populated...");
    ros::Rate rate(10); // 10 Hz
    bool joint_states_received = false;
    ros::Time start_time = ros::Time::now();

    while (ros::ok() && !joint_states_received) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);

            // Check if all required joints are found in the joint_states message
            bool all_joints_found = true;
            for (const auto& joint_name : joint_names) {
                if (std::find(current_joint_states.name.begin(), current_joint_states.name.end(), joint_name) == current_joint_states.name.end()) {
                    all_joints_found = false;
                    break;
                }
            }

            // Verify positions are populated for all joints
            if (all_joints_found && current_joint_states.position.size() >= joint_names.size()) {
                joint_states_received = true;
                ROS_INFO("All joint_states received. Proceeding with movement commands.");
                break;
            }
        }

        // Check if timeout has been reached
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            ROS_ERROR("Timeout reached while waiting for joint_states.");
            return false; // Return false if joint_states are not received within timeout
        }

        rate.sleep(); // Sleep to maintain loop rate
    }

    return joint_states_received;
}

// Function to wait for the IK service to become available
bool waitForIKService(const std::string& service_name, int timeout_ms) {
    // Wait for the service with a timeout
    if (!ros::service::waitForService(service_name, ros::Duration(timeout_ms / 1000.0))) {
        ROS_WARN("%s service is not available after %d milliseconds.", service_name.c_str(), timeout_ms);
        return false; // Return false if the service is not available within the timeout
    }

    ROS_INFO("%s service is available.", service_name.c_str());
    return true; // Return true if the service is available
}

// Callback function to handle updates from the joint states topic
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Lock the mutex to ensure thread-safe access to the joint_states variable
    std::lock_guard<std::mutex> lock(data_mutex);

    // Update the global variable with the latest joint state information
    current_joint_states = *msg;

    // Optional: Add a log statement to debug joint state updates (throttled to avoid flooding)
    ROS_DEBUG_THROTTLE(5, "Joint states updated.");
}

// Call IK service with a retry mechanism if service is temporarily unavailable
bool callIKService(ros::ServiceClient &ik_client, ik_service::PoseIK &ik_srv)
{
    int attempt = 0;
    const int max_attempts = 5;      // Maximum retry attempts
    const int wait_duration = 10000; // 10-second wait between attempts

    while (attempt < max_attempts)
    {
        if (ik_client.call(ik_srv))
        {
            ROS_INFO("IK service call succeeded on attempt %d.", attempt + 1);
            return true;
        }
        else
        {
            ROS_WARN("IK service call failed on attempt %d.", attempt + 1);
            ros::Duration(wait_duration / 1000.0).sleep(); // Wait before retrying
            attempt++;
        }
    }

    ROS_ERROR("Failed to call IK service after %d attempts.", max_attempts);
    return false; // Return false if all attempts fail
}

void sendTrajectory(const std::vector<std::string> &joint_names,ros::Publisher& trajectory_pub,
                    const std::vector<double> &joint_positions) {
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("arm_controller/follow_joint_trajectory", true);

    //ROS_INFO("Waiting Action Server...");
    //arm_client.waitForServer();

    // Crear el mensaje de trayectoria
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;

    // Añadir el punto final con las posiciones deseadas
    trajectory_msgs::JointTrajectoryPoint start_point, target_point;
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        // Set start point to current joint states
        for (const auto& joint_name : joint_names) {
            auto it = std::find(current_joint_states.name.begin(), current_joint_states.name.end(), joint_name);
            if (it != current_joint_states.name.end()) {
                size_t index = std::distance(current_joint_states.name.begin(), it);
                start_point.positions.push_back(current_joint_states.position[index]);
            } else {
                ROS_ERROR("Joint name '%s' not found in joint states.", joint_name.c_str());
                return;
            }
        }
        start_point.time_from_start = ros::Duration(0.0);

        // Set target point to IK solution
        target_point.positions = joint_positions;
        target_point.time_from_start = ros::Duration(2.0); // 2 seconds to reach the target
    }

    traj.points.push_back(start_point);
    traj.points.push_back(target_point);
    traj.header.stamp = ros::Time::now();

    // Publish the trajectory
    trajectory_pub.publish(traj);
    ROS_INFO("Trajectory command published.");
}



bool isArmStationary(const sensor_msgs::JointState& joint_states) {
    double velocity_threshold = 0.01; // Velocidad mínima para considerar "estacionario"
    for (const auto& velocity : joint_states.velocity) {
        if (std::abs(velocity) > velocity_threshold) {
            return false; // El brazo aún está en movimiento
        }
    }
    return true; // El brazo está estacionario
}

// void goalActiveCallback() { 
//     ROS_INFO("The goal is active");
// }

// void feedbackCallback(const control_msgs::JointTrajectoryFeedbackConstPtr& fb) {
//     ROS_INFO("Feedback recived ");
//     for (size_t i = 0; i < fb->joint_names.size(); ++i) {
//         ROS_INFO("  Joint: %s,  Psition: %f", fb->joint_names[i].c_str(), fb->actual.positions[i]);
//     }
// }

// void resultCallback(const actionlib::SimpleClientGoalState& state,
//                     const control_msgs::JointTrajectoryResultConstPtr& res) {
//     ROS_INFO("The action is complete %s", state.toString().c_str());
//     if (res) {
//         ROS_INFO("Result %d .", res->trajectory.points.size());
//     } else {
//         ROS_WARN("No result.");
//     }
// }
// Function to create target poses
std::vector<geometry_msgs::Pose> createTargetPoses()
{
    std::vector<geometry_msgs::Pose> poses;
    poses.clear();

    geometry_msgs::Pose pose;

    // Pose 1
    pose.position.x = 0.75;
    pose.position.y = 0.0;
    pose.position.z = 0.95;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);

    // Pose 2
    pose.position.x = 0.0;
    pose.position.y = 0.75;
    pose.position.z = 0.95;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);

    // Pose 3
    pose.position.x = 0.25;
    pose.position.y = 0.75;
    pose.position.z = 0.95;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);

    // Pose 4
    pose.position.x = 0.75;
    pose.position.y = 0.25;
    pose.position.z = 0.95;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);

    // Pose 5
    pose.position.x = 0.75;
    pose.position.y = 0.45;
    pose.position.z = 0.75;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    poses.push_back(pose);

    return poses;
}


void moveToPoses(ros::NodeHandle &nh,  tf2_ros::Buffer& tfBuffer,  ros::Publisher trajectory_pub) {
    // Obtener las poses objetivo
    //control_msgs::FollowJointTrajectoryGoal goal;
    std::vector<geometry_msgs::Pose> poses = createTargetPoses();

    
    //control_msgs::FollowJointTrajectoryGoal goal;


    //Ik service client 
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");
    // Obtener nombres de las juntas
    std::vector<std::string> joint_names;
    if (!getJointNames(joint_names)) {
        ROS_ERROR("No se pudieron obtener los nombres de las juntas. Abortando.");
        return;
    }

    
    // Preparar solicitud al servicio IK
    // ik_service::PoseIK ik_srv;
    // ik_srv.request.target_pose = a_pose;

    //     // Llamar al servicio IK con reintentos
    //     if (!callIKService(ik_client, ik_srv) || ik_srv.response.num_solutions == 0) {
    //         ROS_WARN("No se encontraron soluciones IK para la pose (%.2f, %.2f, %.2f).",
    //                  a_pose.position.x, a_pose.position.y, a_pose.position.z);
    //         continue;
    //     }

    // std::vector<std::string> joint_names;
    // if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
    // ROS_ERROR("Failed to retrieve joint names from the parameter server.");
    // return;
    //         }
        

    for (const auto &target_pose : poses) {
        ik_service::PoseIK ik_srv;
        ik_srv.request.target_pose = target_pose;

        if(!callIKService(ik_client,ik_srv)){
            ROS_WARN("Skipping pose (%.2f, %.2f, %.2f) due to IK failure.", 
                     target_pose.position.x, target_pose.position.y, target_pose.position.z);
            continue;
        }

        std::vector<double> joint_angles(ik_srv.response.joint_solutions[0].joint_angles.begin(),
                                  ik_srv.response.joint_solutions[0].joint_angles.end());

        

        sendTrajectory(joint_names,trajectory_pub,joint_angles);
        // Wait for the arm to reach the pose
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                if (std::all_of(current_joint_states.velocity.begin(), current_joint_states.velocity.end(),
                                [](double vel) { return std::abs(vel) < 0.01; })) {
                    ROS_INFO("Arm has reached pose (%.2f, %.2f, %.2f).", 
                             target_pose.position.x, target_pose.position.y, target_pose.position.z);
                      //feedbackCallback(goal);       
                    break;
                }
            }
               rate.sleep();
        }

       
        
    }
  
}


bool retrieveJointNames(std::vector<std::string> &joint_names) {
    // Fetch joint names from the parameter server
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("Failed to retrieve joint names from the parameter server.");
        return false;
    }

    // Log the retrieved joint names
    ROS_INFO("Successfully retrieved joint names:");
    for (const auto &joint : joint_names) {
        ROS_DEBUG(" - %s", joint.c_str());
    }

    return true;
}

void retrievePartLocation(ros::NodeHandle &nh, const std::string &product_type) {
    ros::ServiceClient client = nh.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    osrf_gear::GetMaterialLocations srv;
    srv.request.material_type = product_type;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call material_locations service for product type: %s", product_type.c_str());
        return;
    }

    for (const auto &unit : srv.response.storage_units) {
        if (unit.unit_id == "belt") {
            ROS_WARN_STREAM("Skipping belt location for product type: " << product_type);
            continue;
        }

        ROS_INFO_STREAM("Product type: " << product_type << " located in unit: " << unit.unit_id);

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (product_locations.count(product_type)) {
                for (const auto &part_location : product_locations[product_type]) {
                    const auto &pose = part_location.part_pose;
                    ROS_DEBUG_STREAM("Location details - x: " << pose.position.x
                                                             << ", y: " << pose.position.y
                                                             << ", z: " << pose.position.z);
                }
            } else {
                ROS_WARN_STREAM("No logical camera data found for product type: " << product_type);
            }
        }
    }
}

bool transformPoseToArmFrame(const geometry_msgs::Pose &camera_pose, const std::string &source_frame, geometry_msgs::PoseStamped &arm_pose, tf2_ros::Buffer &tfBuffer) {
    try {
        // Get the transform from the source frame to the arm's base frame
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("arm1_base_link", source_frame, ros::Time(0), ros::Duration(1.0));

        // Create a stamped pose in the source frame
        geometry_msgs::PoseStamped stamped_camera_pose;
        stamped_camera_pose.pose = camera_pose;
        stamped_camera_pose.header.frame_id = source_frame;

        // Transform the pose to the arm frame
        tf2::doTransform(stamped_camera_pose, arm_pose, transform);

        // Lift Z position slightly to avoid collisions
        arm_pose.pose.position.z += 0.10;

        ROS_DEBUG_STREAM("Transformed pose to arm frame: x = " << arm_pose.pose.position.x
                                                               << ", y = " << arm_pose.pose.position.y
                                                               << ", z = " << arm_pose.pose.position.z);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed to transform pose from %s to arm1_base_link: %s", source_frame.c_str(), ex.what());
        return false;
    }
}

void publishTrajectory(sensor_msgs::JointState& joint_states, ros::Publisher& trajectory_pub) {
    trajectory_msgs::JointTrajectory joint_trajectory;

    // Nombres de las juntas
    joint_trajectory.joint_names = {
        "linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint",
        "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    // Configurar un punto intermedio
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(joint_trajectory.joint_names.size());

    // Establecer posiciones iniciales desde `joint_states`
    for (size_t t_joint = 0; t_joint < joint_trajectory.joint_names.size(); ++t_joint) {
        for (size_t s_joint = 0; s_joint < joint_states.name.size(); ++s_joint) {
            if (joint_trajectory.joint_names[t_joint] == joint_states.name[s_joint]) {
                point.positions[t_joint] = joint_states.position[s_joint];
                break;
            }
        }
    }

    // Modificar el ángulo del codo (elbow_joint, índice 3)
    point.positions[3] += 0.1; // Ajustar en 0.1 radianes

    // Configurar el actuador lineal
    point.positions[0] = joint_states.position[1];

    // Establecer el tiempo de duración del movimiento
    point.time_from_start = ros::Duration(0.25);

    // Agregar el punto a la trayectoria
    joint_trajectory.points.push_back(point);

    // Configurar el encabezado del mensaje
    static int count = 0;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "arm1_base_link";

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_trajectory.joint_names;
    joint_state.position = joint_trajectory.points[0].positions;// Publicar la trayectoria
    ROS_INFO("Publicando trayectoria...");
    trajectory_pub.publish(joint_trajectory);
    
// Crear el objetivo (Goal) para el Action Server
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.header.frame_id = "base_link";  // El marco de referencia puede variar

    // Llenar la trayectoria con las posiciones de las articulaciones
    goal.trajectory.points.resize(1);  // Solo un punto en este ejemplo
    goal.trajectory.points[0].positions.push_back(1.57);  // Ejemplo: posición para la articulación 1 (en radianes)
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);  // Tiempo para alcanzar este punto

    // Enviar el objetivo al Action Server con los callbacks
    trajectory_ac.sendGoal(goal, &resultCallback, &goalActiveCallback, &feedbackCallback);

    // Esperar hasta que el objetivo esté completo
    trajectory_ac.waitForResult();
    ROS_INFO("Final state: %s", trajectory_ac.getState().toString().c_str());
}




// Process the orders
void processOrders(ros::NodeHandle &nh, ros::ServiceClient &ik_client,  ros::Publisher trajectory_pub, tf2_ros::Buffer& tfBuffer) {
    // Definir restricciones de ángulos de las juntas
    const double SHOULDER_LIFT_MIN = -M_PI / 4;     // -45 grados
    const double SHOULDER_LIFT_MAX = M_PI / 4;      // +45 grados
    const double WRIST_2_NEAR_PI_2 = M_PI / 2;      // Cerca de π/2
    const double WRIST_2_NEAR_3PI_2 = 3 * M_PI / 2; // Cerca de 3π/2
    const double WRIST_2_TOLERANCE = 0.1;           // Tolerancia para wrist_2
    
    // Retrieve joint names from parameter server
    std::vector<std::string> joint_names;
    if (!retrieveJointNames(joint_names)) {
        ROS_ERROR("Failed to retrieve joint names. Aborting processOrders.");
        return;
    }

    // Main loop to process orders
    while (ros::ok()) {
        // Check for available orders
        if (order_vector.empty()) {
            ROS_INFO_THROTTLE(10, "Waiting for orders...");
            ros::Duration(0.5).sleep();
            continue;
        }

        // Lock and retrieve the current order
        osrf_gear::Order current_order;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_order = order_vector.front();
        }

        ROS_INFO("Processing order: %s", current_order.order_id.c_str());

        // Loop through each shipment in the order
        for (const auto &shipment : current_order.shipments) {
            ROS_INFO("Processing shipment type: %s", shipment.shipment_type.c_str());

            // Loop through each product in the shipment
            for (const auto &product : shipment.products) {
                ROS_INFO("Processing product type: %s", product.type.c_str());

                // Locate part using logical camera and material locations
                retrievePartLocation(nh, product.type);

                // Retrieve part pose from logical camera data
                geometry_msgs::Pose part_pose;
                std::string camera_frame;

                {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    if (product_locations.find(product.type) == product_locations.end() || product_locations[product.type].empty()) {
                        ROS_WARN("No available parts of type: %s found in product_locations.", product.type.c_str());
                        continue;
                    }
                    // Get the first available part
                    part_pose = product_locations[product.type].front().part_pose;
                    camera_frame = product_locations[product.type].front().source_frame_id;
                }

                // Transform the part pose to the robot arm frame
                geometry_msgs::PoseStamped goal_pose;
                transformPoseToArmFrame(part_pose, camera_frame, goal_pose, tfBuffer);

                // Prepare IK service request
                ik_service::PoseIK ik_srv;
                ik_srv.request.target_pose = goal_pose.pose;

                // Call IK service to get joint solutions
                if (!callIKService(ik_client, ik_srv)) {
                    ROS_ERROR("Failed to call IK service for product type: %s", product.type.c_str());
                    continue;
                }

                // Select a valid IK solution based on constraints
                std::vector<double> selected_solution;
                bool valid_solution_found = false;

                for (int i = 0; i < ik_srv.response.num_solutions; ++i) {
                    const auto &solution = ik_srv.response.joint_solutions[i].joint_angles;

                    // Check joint constraints
                    double shoulder_lift_joint = solution[1]; // Index for shoulder_lift_joint
                    double wrist_2_joint = solution[4];       // Index for wrist_2_joint

                    if (shoulder_lift_joint < SHOULDER_LIFT_MIN || shoulder_lift_joint > SHOULDER_LIFT_MAX) {
                        continue; // Skip invalid solutions
                    }

                    if (std::abs(wrist_2_joint - WRIST_2_NEAR_PI_2) > WRIST_2_TOLERANCE &&
                        std::abs(wrist_2_joint - WRIST_2_NEAR_3PI_2) > WRIST_2_TOLERANCE) {
                        continue; // Skip invalid solutions
                    }

                    // Select the first valid solution
                    selected_solution = std::vector<double>(solution.begin(), solution.end());
                    valid_solution_found = true;
                    break;
                }

                if (!valid_solution_found) {
                    ROS_WARN("No valid IK solution found for product type: %s", product.type.c_str());
                    continue;
                }

                // Execute the selected IK solution by publishing a trajectory
                sendTrajectory(joint_names, trajectory_pub, selected_solution);

                // Wait for the robot arm to reach the target position
                ros::Rate rate(10); // 10 Hz
                while (ros::ok()) {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    bool arm_at_rest = std::all_of(current_joint_states.velocity.begin(), current_joint_states.velocity.end(),
                                                   [](double vel) { return std::abs(vel) < 0.01; });
                      if (arm_at_rest) {
                        ROS_INFO("Robot arm has reached the target position for product type: %s", product.type.c_str());
                        //resultCallback(actionlib, goal);
                        break;
                    }
                    rate.sleep();
                }
            }
        }

        // Remove the processed order from the queue
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            order_vector.erase(order_vector.begin());
        }

        ROS_INFO("Completed processing order: %s", current_order.order_id.c_str());
    
    }
}




int main(int argc, char **argv) {
    // Inicializar el nodo ROS
   // ROS_INFO("Hola");
    ros::init(argc, argv, "competition_controller_node");
    ros::NodeHandle nh;

   
    // Call the startCompetition function and handle success or failure
    if (!startCompetition(nh)) {
        ROS_ERROR("Unable to start '/ariac/start_competition'. Shutting down node.");
        ros::shutdown();
    }
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer); 

    order_vector.clear();
    product_locations.clear();
    joint_names.clear();

    ros::Duration(1.0).sleep();

    if (!getJointNames(joint_names)) {
        ROS_ERROR("Joint name retrieval or validation failed. Exiting...");
        return 1;
    }

    if (!waitForJointStates(joint_names)) {
        ROS_ERROR("Failed to receive joint_states. Exiting...");
        return 1;
    }

    ROS_INFO("'/ariac/start_competition' service is starting please wait!");
    ros::service::waitForService("/ariac/start_competition");

  


    ros::Subscriber order_subscriber = nh.subscribe("/ariac/orders", 10, orderCallback);
    // Suscribirse a las cámaras lógicas
    subscribeToCameras(nh);
ROS_INFO("Hola despues de suscribir camaras");
      if (!waitForIKService("pose_ik", 10000)) {
        ROS_ERROR("IK service is unavailable. Exiting...");
        return 1; // Exit if the IK service is not ready
      }
    trajectory_ac.waitForServer();
    ROS_INFO("Connected to action server.");

    // // Ahora puedes enviar objetivos de acción
    // trajectory_msgs::JointTrajectory joint_trajectory;
    // // Aquí, deberías llenar el joint_trajectory con las posiciones de las juntas

    // publishTrajectory(joint_trajectory); 
    trajectory_msgs::JointTrajectory joint_trajectory;
    // Inicializar `joint_trajectory` 

    sensor_msgs::JointState joint_states;
    joint_states.name = joint_trajectory.joint_names;
    joint_states.position = joint_trajectory.points[0].positions; // Usa el primer punto
    

    // Definir el publisher para las trayectorias
    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

    // Suscriptor para los estados de las juntas
    ros::Subscriber joint_states_sub = nh.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);

    // Cliente para el servicio IK
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");

    publishTrajectory(joint_states, trajectory_pub);

    
    //
    //
    //
    // cuanto llame initializeROS poner esto:
        // IK service client
    ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");
    ROS_INFO("Initialized IK service client for 'pose_ik'.");

    // Publisher for sending joint trajectories
    trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
    ROS_INFO("Initialized joint trajectory publisher for '/ariac/arm1/arm/command'.");

    // Subscriber for joint states
    joint_states_sub = nh.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);
    ROS_INFO("Initialized joint states subscriber for '/ariac/arm1/joint_states'.");

    // Start the AsyncSpinner
    static ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    ROS_INFO("Async spinner started with 1 thread.");



    // Esperar a que el servicio IK esté disponible
    if (!ros::service::waitForService("pose_ik", 10000)) {
        ROS_WARN("El servicio pose_ik no está disponible después de 10 segundos.");
    } else {
        ROS_INFO("El servicio pose_ik está disponible.");
    }

    // Iniciar la competencia
    if (!startCompetition(nh)) {
        ROS_ERROR("No se pudo iniciar la competencia. Abortando.");
        return 1;
    }

    // Llamar a moveToPoses para mover el brazo a las posiciones objetivo
    moveToPoses(nh, tfBuffer, trajectory_pub);

    //ros::spinOnce();
    // Procesar órdenes
    processOrders(nh, ik_client, trajectory_pub, tfBuffer);

    ROS_INFO("Finalizando el nodo.");
    return 0;
}


