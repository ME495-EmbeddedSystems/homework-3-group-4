# Homework 3 (Group 4)

Authors: Ben Benyamin, David Khachatryan, Pushkar Dave, Haodong Wang, Sairam Umakanth  

This package creates an API using multiple different classes that offers an interface
for the user to manipulate a robotic system. An example of a user node is implemented via
the pick_node, which operates a pick and place functionality on a simple object, avoiding
an obstacle.

## Quickstart
1. After sourcing franka workspace, use `ros2 launch franka_fer_moveit_config demo.launch.py` to initialize franka and rviz
2. Use `ros2 launch object_mover object_mover.launch.py` to launch the pick node
3. Run `ros2 service call /pick object_mover_interfaces/srv/PickPose "{pick_point: {position: {x: 0.49, y: 0.13, z: 0.03}, orientation: {x: 3.14, y: 3.14, z: 0.0, w: 0.0}}}"` to trigger the pick service to enable pick task
4. Here is a video of the pick task conducted on the real franka panda robot

[group4_hw3_demo.webm](https://github.com/user-attachments/assets/962561f6-b9c3-4066-9daa-0b189d6e9769)