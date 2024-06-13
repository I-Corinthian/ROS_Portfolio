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
**Description**: Demonstrates navigation and obstacle avoidance for TurtleBot3 in a simulated environment using the ROS 2 navigation stack and Gazebo.

**Preview**
![Turtlebot3 Navigation Sim](ros_portfolio/nav2_sim/ros_ws/src/turtle_nav_sim/img/project2.png)

Each project includes detailed documentation, setup instructions, and usage examples to help you understand and replicate the work. Feel free to explore the repositories and try out the projects on your own setup!

Thank you for visiting my portfolio. If you have any questions or feedback, please don't hesitate to reach out.
