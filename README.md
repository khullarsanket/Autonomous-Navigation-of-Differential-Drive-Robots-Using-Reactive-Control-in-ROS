# Autonomous Navigation of Differential Drive Robots using Reactive Control in ROS

## Project Overview
This project develops an autonomous navigation system for differential drive robots, utilizing ROS for simulation. By integrating an occupancy grid map and a LiDAR sensor, the simulation's fidelity and utility are significantly enhanced. The system comprises two ROS nodes: the Differential Drive Simulator Node for simulating robot movement and the Navigation Controller Node for dynamic obstacle avoidance and path planning.

## Key Features
- **Reactive Control:** Real-time environment adaptation for dynamic and unpredictable settings.
- **Enhanced Safety and Adaptability:** Maintains safe distances from obstacles, suitable for various scenarios including narrow corridors.
- **Laser Scan Data Segmentation:** Divides data into distinct regions for effective decision-making and course adjustment.

## System Design
The Differential Drive Simulator Node simulates the robot's physical characteristics and environment, using velocity commands to calculate its pose and simulate laser scan data for perception. The Navigation Controller Node processes this data to control the robot's movements, employing a reactive control strategy to navigate and avoid collisions.

## Operational Flow
1. **Sensing and Perception:** Generates laser scan data simulating obstacle detection.
2. **Decision Making:** Processes laser data to navigate safely, avoiding obstacles.
3. **Motion Control:** Executes movement commands based on the navigation strategy.
4. **Feedback Loop:** Updates robot pose and validates collision avoidance, enhancing the navigation process.

![image](https://github.com/khullarsanket/Autonomous-Navigation-of-Differential-Drive-Robots-Using-Reactive-Control-in-ROS/assets/119709438/c2951a03-d458-4981-8bc1-add728c43e8d)

![image](https://github.com/khullarsanket/Autonomous-Navigation-of-Differential-Drive-Robots-Using-Reactive-Control-in-ROS/assets/119709438/1b0f7bdc-0495-422a-96d0-7ac2e4166c44)



