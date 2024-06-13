# ROS2 Robotics Portfolio

Welcome to my ROS 2 Robotics Portfolio! This repository showcases a series of projects I have developed to demonstrate my skills and expertise in robotics using the Robot Operating System (ROS 2). Each project is designed to highlight different aspects of robotics, including simulation, control, navigation, and computer vision.

## Projects

### 1. ROS2 Differential Drive Robot Simulation
**Description:**  
A simple URDF model of a differential drive robot, simulated in Gazebo and controlled via keyboard teleoperation. This project includes a custom-made URDF specifically designed to work seamlessly with Gazebo. Additionally, I have written a launch file to facilitate the easy opening of the simulation environment in Gazebo. To control the robot, you can use the `teleop_keyboard` package.

#### 1. Launch Simulation 

   ```bash
   ros2 launch dds_package launch_gazebo_sim.launch.py
   ```
#### 2. Run Teleop_Keybord

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

**Skills Expressed:**
- **URDF Modeling:** Custom creation of a URDF (Unified Robot Description Format) file to define the robot's physical and visual properties.
- **Simulation:** Setting up and running simulations in Gazebo to test and visualize robot behaviors.
- **Launch Files:** Writing ROS 2 launch files to streamline the process of starting up the robot simulation.
- **Teleoperation:** Implementing keyboard teleoperation to manually control the robot, showcasing knowledge of ROS 2 topic publishing and subscribing.

**Preview**
![Differential Drive Robot Sim](ros_portfolio/differential_drive_sim/ros_ws/src/dds_package/img/project1.png)

### 2. TurtleBot3 Simulation and Navigation
**Description:**  
This project demonstrates navigation and obstacle avoidance for TurtleBot3 in a simulated environment using the ROS 2 navigation stack and Gazebo. It utilizes custom launch files to start mapping and subsequently use the generated map for navigation.

**Key Features:**
- **Simulation Environment:** Uses Gazebo to simulate the TurtleBot3 and its environment, providing a realistic testbed for developing and testing navigation algorithms.
- **Custom Launch Files:** Custom launch files are created to facilitate the mapping process, allowing for easy setup and execution of the mapping and navigation tasks.
- **Mapping:** Implements SLAM (Simultaneous Localization and Mapping) to generate a map of the environment, which is crucial for effective navigation and obstacle avoidance.
- **Navigation:** Utilizes the generated map to navigate through the environment, demonstrating path planning, real-time obstacle avoidance, and goal-reaching capabilities.

**Skills Demonstrated:**
- **ROS 2 Proficiency:** Showcases advanced knowledge of ROS 2, including working with nodes, topics, services, and the ROS 2 navigation stack.
- **Robot Simulation:** Experience in setting up and configuring robot simulations using Gazebo.
- **SLAM Techniques:** Practical application of SLAM for real-time mapping and localization in a dynamic environment.
- **Navigation and Control:** Development of navigation strategies, including path planning and obstacle avoidance.
- **Launch File Creation:** Proficiency in creating and managing custom ROS 2 launch files to streamline complex robot operations.
- **Problem-Solving:** Demonstrates the ability to integrate multiple components and solve practical robotics problems.

#### 1. Launch Map genration 

   ```bash
   ros2 launch turtle_nav_sim slam_map_genarate.launch.py
   ```
   ```bash
   ros2 run nav2_map_server map_saver_cli -f {path to save the genrated map}
   ```
#### 2. launch Turtlebot3 sim for navigation

   ```bash
   ros2 launch turtle_nav_sim nav2_sim.launch.py 
   ```   

- The above Launch file assumes the path the map was saves is [ros_portfolio/nav2_sim/ros_ws/src/turtle_nav_sim/map]

**Preview**
![Turtlebot3 Navigation Sim](ros_portfolio/nav2_sim/ros_ws/src/turtle_nav_sim/img/project2.png)

Each project includes detailed documentation, setup instructions, and usage examples to help you understand and replicate the work. Feel free to explore the repositories and try out the projects on your own setup!

Thank you for visiting my portfolio. If you have any questions or feedback, please don't hesitate to reach out.
