# vlm_f1tenth

to receive llm_token from mac, run this in f1tenth car

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

ros2 launch stanley_control stanley_launch.py

ros2 run decision_maker decision_node

ros2 launch particle_filter localize_launch.py

python3 vlm_client_node.py

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed

ros2 launch f1tenth_stack bringup_launch.py
