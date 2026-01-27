# Foundational Models-based Autonomous Driving on F1TENTH


This repository provides a **ROS 2–based autonomous driving software stack for the F1TENTH platform**, built around **foundational models (VLM/LLM)** for high-level decision making, combined with classical perception, localization, and control modules.

![System Architecture](images\system_architecture.png)


The system follows a layered architecture:
- **Perception & Localization**: ZED camera + particle filter
- **Control**: Stanley controller
- **Decision Making**: Rule/logic-based node augmented with VLM/LLM reasoning
- **Interface**: ROS bridge for external LLM communication

---

## Prerequisites

- ROS 2 (installed and sourced)
- A built ROS 2 workspace (example: `~/ros2_ws`)
- ZED camera and `zed_wrapper` installed
- Network connectivity if using an external LLM (e.g., Mac → car)

Build and source your workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
1. Bring up the F1TENTH base stack

This initializes core nodes, TFs, and vehicle interfaces.
```bash
ros2 launch f1tenth_stack bringup_launch.py
```

2. Launch the ZED Camera

Starts the ZED camera driver and publishes depth/image perception topics.
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```

3. Run Particle Filter Localization
```bash
ros2 launch particle_filter localize_launch.py
```

4. Run the Stanley Controller

Handles low-level lateral control and publishes steering commands to the car.
```bash
ros2 launch stanley_control stanley_launch.py
```
5. Run the Decision-Making Node

Executes the high-level driving logic and updates controller behavior based on environmental context.
```bash
ros2 run decision_maker decision_node
```

6. Run the VLM Client Node

Connects the local ROS 2 system with the Vision/Language Model pipeline. This should be run from the scripts directory:
```bash
cd ~/ros2_ws/scripts
python3 vlm_client_node.py
```

External Communication (Mac → F1TENTH Car)

If you are offloading reasoning to an external machine (e.g., a Mac Pro or Laptop), you must run the ROS bridge on the F1TENTH car to enable communication:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
This bridge allows WebSocket-based communication between the external VLM client and the ROS 2 ecosystem on the vehicle.

Multi-Device Networking: Ensure all machines are on the same local network and are using the same ROS_DOMAIN_ID. If nodes cannot "see" each other, check your firewall settings and export the domain ID: export ROS_DOMAIN_ID=X.

