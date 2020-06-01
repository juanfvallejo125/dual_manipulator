To run the teleoperation simulation run the following commands
1. Run the arm_manipulator controller with the following command: roslaunch arm_manipulator_controller arm_manipulator_controller.launch use_platform:=false
2. Run the dual_manipulator_teleop with the follwing command: roslaunch dual_manipulator_teleop_keyboard.launch
3. Command the robot with the keys indicated in the prompt
4. Can be run with gazebo model too if it is launched, otherwise just monitor the command topics of each joint
