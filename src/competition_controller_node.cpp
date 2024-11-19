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
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/JointState.h"

// Global variable to store th e current state of the joints 
sensor_msgs::JointState current_joint_states;
ros::Time last_joint_state_time;
// Global vector to store orders 
std::vector<osrf_gear::Order> order_vector;

ros::Publisher trajectory_pub;


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
   // ros::spinOnce(); 
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

bool getJointNames(std::vector<std::string>& joint_names) {
    // Obtener los nombres de las juntas desde el servidor de parámetros
    if (!ros::param::get("/ariac/arm1/arm/joints", joint_names)) {
        ROS_ERROR("No se pudieron obtener los nombres de las juntas del servidor de parámetros.");
        return false;
    }
    ROS_INFO("Nombres de las juntas obtenidos:");
    for (const auto &joint : joint_names) {
        ROS_INFO(" - %s", joint.c_str());
    }
    return true;
}

void sendTrajectory(const std::vector<std::string>& joint_names, const std::vector<std::vector<double>>& trajectory_points) {
    // Configurar el cliente para el Action Server del UR10
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_client("arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Esperando al Action Server...");
    arm_client.waitForServer();

    // Crear el mensaje de trayectoria
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;

    // Añadir puntos de trayectoria
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = trajectory_points[i];
        point.time_from_start = ros::Duration(2.0 * (i + 1)); // Incrementar el tiempo por cada punto
        traj.points.push_back(point);
    }

    // Crear el objetivo y enviarlo al Action Server
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;

    ROS_INFO("Enviando la trayectoria al UR10...");
    arm_client.sendGoal(goal);

    arm_client.waitForResult();

    if (arm_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("El UR10 completó la trayectoria con éxito.");
    } else {
        ROS_WARN("El UR10 no pudo completar la trayectoria.");
    }
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    current_joint_states = *msg;
    last_joint_state_time = ros::Time::now();
    
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

void publishTrajectory(const sensor_msgs::JointState& joint_states) {
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

    // Publicar la trayectoria
    ROS_INFO("Publicando trayectoria...");
    trajectory_pub.publish(joint_trajectory);
}

void moveToPoses(ros::NodeHandle &nh) {
    // Configurar cliente del servicio IK
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");
    if (!ros::service::waitForService("pose_ik", ros::Duration(3.0))) {
        ROS_ERROR("IK service 'pose_ik' is not available. Exiting...");
        return;
    }

    // Inicializar el vector de poses
    std::vector<geometry_msgs::Pose> poses;
    poses.clear();

    // Agregar las poses especificadas
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0; pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0;

    pose.position.x = 0.75; pose.position.y = 0.0; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.0; pose.position.y = 0.75; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.25; pose.position.y = 0.75; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.75; pose.position.y = 0.25; pose.position.z = 0.95; poses.push_back(pose);
    pose.position.x = 0.75; pose.position.y = 0.45; pose.position.z = 0.75; poses.push_back(pose);

    // Obtener nombres de juntas
    std::vector<std::string> joint_names;
    if (!getJointNames(joint_names)) {
        ROS_ERROR("No se pudieron obtener los nombres de las juntas. Abortando.");
        return;
    }

    for (const auto& a_pose : poses) {
        // Preparar solicitud al servicio IK
        ik_service::PoseIK ik_srv;
        ik_srv.request.target_pose = a_pose;

        // Llamar al servicio IK
        if (!ik_client.call(ik_srv)) {
            ROS_ERROR("Failed to call IK service for pose (%.2f, %.2f, %.2f)", 
                      a_pose.position.x, a_pose.position.y, a_pose.position.z);
            continue;
        }

        if (!ik_srv.response.success) {
            ROS_WARN("No valid IK solution found for pose (%.2f, %.2f, %.2f)", 
                     a_pose.position.x, a_pose.position.y, a_pose.position.z);
            continue;
        }

        // Generar trayectoria
        trajectory_msgs::JointTrajectory joint_trajectory;
        joint_trajectory.joint_names = joint_names;

        // Punto inicial: Estado actual del UR10
        trajectory_msgs::JointTrajectoryPoint start_point;
        start_point.positions = current_joint_states.position;
        start_point.time_from_start = ros::Duration(0.0);

        // Punto final: Solución IK
        trajectory_msgs::JointTrajectoryPoint goal_point;
        goal_point.positions = ik_srv.response.joint_solutions[0].joint_angles; // Elegir la primera solución
        goal_point.time_from_start = ros::Duration(2.0); // 2 segundos para moverse

        joint_trajectory.points.push_back(start_point);
        joint_trajectory.points.push_back(goal_point);

        // Publicar la trayectoria
        joint_trajectory.header.stamp = ros::Time::now();
        trajectory_pub.publish(joint_trajectory);

        ROS_INFO("Published trajectory to pose (%.2f, %.2f, %.2f)", 
                 a_pose.position.x, a_pose.position.y, a_pose.position.z);

        // Esperar hasta que el UR10 esté estacionario
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            if (isArmStationary(current_joint_states)) {
                ROS_INFO("UR10 has reached pose (%.2f, %.2f, %.2f)", 
                         a_pose.position.x, a_pose.position.y, a_pose.position.z);
                break;
            }
            rate.sleep();
        }
    }
}



// Process the orders
void processOrders(ros::NodeHandle &nh) {
    // Verificar disponibilidad del servicio IK
    if (!ros::service::waitForService("pose_ik", ros::Duration(3.0))) {
        ROS_ERROR("IK service 'pose_ik' is not available. Exiting...");
        return;
    }

    // Configurar cliente para el servicio IK
    ros::ServiceClient ik_client = nh.serviceClient<ik_service::PoseIK>("pose_ik");
    ik_service::PoseIK ik_srv;

    // Inicializar el vector de nombres de juntas
    std::vector<std::string> joint_names;
    if (!getJointNames(joint_names)) {
        ROS_ERROR("No se pudieron obtener los nombres de las juntas. Abortando.");
        return;
    }

    // Loop principal para procesar órdenes
    while (ros::ok()) {
        if (current_joint_states.name.empty()) {
            ROS_WARN("Esperando datos de joint_states...");
            ros::Duration(0.1).sleep();
            continue;
        }

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
                        ik_srv.request.target_pose = goal_pose.pose;

                        // Llamar al servicio IK
                        if (ik_client.call(ik_srv)) {
                            if (ik_srv.response.success) {
                                // Restringir las soluciones a una válida (ejemplo: solución más cercana al estado actual)
                                double min_difference = std::numeric_limits<double>::max();
                                std::vector<double> selected_solution;

                                for (const auto& solution : ik_srv.response.joint_solutions) {
                                    double difference = 0.0;
                                    for (size_t i = 0; i < solution.joint_angles.size(); ++i) {
                                        difference += std::abs(solution.joint_angles[i] - current_joint_states.position[i]);
                                    }
                                    if (difference < min_difference) {
                                        min_difference = difference;
                                        selected_solution = solution.joint_angles;
                                    }
                                }

                                // Generar la trayectoria con la solución seleccionada
                                trajectory_msgs::JointTrajectory joint_trajectory;
                                joint_trajectory.joint_names = joint_names;

                                // Punto inicial (estado actual)
                                trajectory_msgs::JointTrajectoryPoint start_point;
                                start_point.positions = current_joint_states.position;
                                start_point.time_from_start = ros::Duration(0.0);

                                // Punto final (solución seleccionada)
                                trajectory_msgs::JointTrajectoryPoint goal_point;
                                goal_point.positions = selected_solution;
                                goal_point.time_from_start = ros::Duration(2.0); // Ajustar el tiempo según sea necesario

                                joint_trajectory.points.push_back(start_point);
                                joint_trajectory.points.push_back(goal_point);

                                // Publicar la trayectoria
                                trajectory_pub.publish(joint_trajectory);
                                ROS_INFO("Trayectoria publicada para mover el UR10.");
                            } else {
                                ROS_WARN("IK service call was successful, but no valid solution found.");
                            }
                        } else {
                            ROS_ERROR("Failed to call IK service.");
                        }
                    }
                }
            }
            order_vector.erase(order_vector.begin());
        }

        // Publicar estados de las juntas periódicamente
        ROS_INFO_STREAM_THROTTLE(10, "Current joint states: " << current_joint_states);
        ROS_INFO_THROTTLE(10, "Last joint state time: %.2f", last_joint_state_time.toSec());

        ros::Duration(0.1).sleep(); // Esperar un momento antes de iterar
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "competition_controller_node");
    ros::NodeHandle nh;
    
    ros::Subscriber joint_states_sub = nh.subscribe("/ariac/arm1/joint_states", 10, jointStatesCallback);

    trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

    //delay
    ros::Duration(2.0).sleep();

    if (!startCompetition(nh)) return 1;
    
    //declare buffer and listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    receiveOrders(nh);
    subscribeToCameras(nh);

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();              // Start the spinner

 
    moveToPoses(nh);
    // Configure the subscriber to recive orders 
  
    // Process the orders, the while is here 
    processOrders(nh);

    return 0;
}