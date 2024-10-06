# ROS Simulated Autonomous Drone Delivery

## Overview

This project is a simulation platform for managing a fleet of drones performing autonomous package deliveries. Built on ROS (Robot Operating System) and simulated in Gazebo, the system allows operators to monitor drone telemetry, assign deliveries, and observe real-time drone paths through an easy-to-use web interface. Although currently designed for simulation, the long-term goal is to make the system adaptable for real-life drone operations from the web interface.

### Key Features

The system offers real-time drone telemetry, providing insights into battery status, GPS location, and the drone’s current task (whether delivering a package or returning to base). Drones autonomously plan and adjust their delivery routes in response to dynamic obstacles using 3D occupancy mapping. The intuitive React and Bootstrap interface makes it easy for operators to manage the fleet without needing deep technical knowledge. To ensure reliability, the system includes a CI/CD pipeline to test all aspects of the system, automating testing and deployment.

## Project Details

The backend is powered by a REST API built with Spring Boot and Hibernate ORM, managing the interaction between drones and deliveries. PostgreSQL stores all relevant data, from delivery statuses to drone telemetry. The simulation utilizes ROS and Gazebo to mimic real-world drone behaviors, including dynamic path planning and obstacle avoidance using MoveIt and OctoMap.

The frontend interface, built with React and Bootstrap, allows operators to manage multiple drones simultaneously. Users can assign deliveries, monitor telemetry data, and make real-time decisions on drone tasks. The interface is designed for ease of use, ensuring that operators can interact with the drones without needing technical expertise.

## Installation

To run this project locally:

1. Clone the repository:
   ```bash
   git clone https://github.com/John075/Warehouse-Delivery.git
   ```

TO-DO - complete this

## Credits
Took the original Hector Quadcopter implementation with MoveIt integrated from [tahsinkose’s repository](https://github.com/tahsinkose/hector-moveit)
