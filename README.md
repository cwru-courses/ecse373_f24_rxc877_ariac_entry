# ecse373_f24_rxc877_ariac_entry
# Lab 5 ariac_entry Package

The ARIAC 2019 Lab package provides the tools necessary for simulating in a warehouse environment. The package uses ROS to interact with robot models and sensors, following the rules of the ARIAC competition.

## Prerequisites
  
  Before using the package in order to run the ARIAC enviroment and UR10 robot, ensure that you have the following dependencies installed:
  
  - ROS Noetic
  - roscpp
  - rviz
  - Gazebo (for simulation)
  - ecse_373_ariac (ARIAC enviroment setup)
  - ik_service (inverse kinematics service)
  - tf2_ros and tf2_geometry_msgs
  - ros-noetic-osrf-gear
  - and ros-noetic-cwru-ecse-373


## Installation of Dependencies

  # Add the GPG key and OSRF repository for Gazebo and ARIAC
  ```bush 
  wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/    apt/sources.list.d/gazebo-stable.list'
  sudo apt-get update
  ```

  # Install the required ROS packages
  ```bush
  sudo apt installros-noetic-osrf-gear ros-noetic-cwru-ecse-373 ros-noetic-joint-state-controller ros-noetic-       effort-controllers ros-noetic-position-controllers
  ```

  Update the enviroment 
  ```bush
  sudo update
  ```

## Installation

Follow the steps below to install and set up the package.

0. **Open ROS noetic in two terminals**
    ```bash
    source /opt/ros/noetic/setup.bash
    source ~/ariac_ws/devel/setup.bash
    ```
1. **In one terminal run the roscore**:

    ```bash
    roscore
    ```
2. **Change the terminal and continue: Clone the repository**:

    ```bash
    git clone git@github.com:cwru_courses/ecse373_f24_rxc877_ariac_entry.git
    ```

    
3. **Navigate to your ROS workspace**:

    ```bash
    cd ~/ariac_ws
    ```

4. **Compile the workspace**:

    ```bash
    catkin_make
    ```

5. **Source your workspace**:

    ```bash
    source devel/setup.bash
    ```

## Steps to Launch the Nodes

To launch the ARIAC simulation, follow these steps:

1. **Launch the ARIAC simulation**:

    ```bash
    roslaunch ariac_enry entry.launch
    ```

2. **Gazebo openning**:
    This should open thr gazebo simulation running the service called start_competition, it is also going to runth eservice of the previous lab /calculate_ik
    

## Example Output

When running the `ariac_simulation.launch`, you should see output similar to the following:

  ```bash
  PARAMETERS
   * /ik_client_node/wait_for_service: /calculate_ik
   * /rosdistro: noetic
   * /rosversion: 1.17.0
  
  NODES
    /
      ariac_sim (ecse_373_ariac/gear.py)
      competition_controller_node (ariac_entry/competition_controller_node)
      ik_client_node (ik_service/ik_client)
      ik_service_node (ik_service/ik_service)
  
  ROS_MASTER_URI=http://localhost:11311
  
  process[ariac_sim-1]: started with pid [16192]
  process[ik_service_node-2]: started with pid [16193]
  process[ik_client_node-3]: started with pid [16195]
  process[competition_controller_node-4]: started with pid [16196]
  [FATAL] [1731434088.445175909]: You must call ros::init() before creating the first NodeHandle
  [ INFO] [1731434088.446538423]: Service ready 
  [ERROR] [1731434088.452031901]: [registerPublisher] Failed to contact master at [:0].  Retrying...
  [ INFO] [1731434088.454736722]: The ik_service has been called
  [ INFO] [1731434088.454878290]: Call to ik_service returned [8] solutions
  [ INFO] [1731434088.455642241]: solution 0:
  ...
  ```
## File structure 
  ```bash
  ariac_entry/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch
  │   └── entry.launch 
  ├── src
    └── competition_controller_node.cpp

  ```

