/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/* Modified by Juan Vallejo */

#ifndef OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
#define OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetGoalCurrent.h"

#define NUM_OF_JOINT 6 //Modified to 6 joints for one arm
#define DELTA 0.01
double JOINT_DELTA = 0.05;
double PATH_TIME = 0.1;

double dt = 0.01;
double dq = 0.005;  

class DualManipulatorTeleop
{
 public:
  DualManipulatorTeleop();
  ~DualManipulatorTeleop();

  // update
  void printText();
  void setGoal(char ch);

  //Added by Juan
  bool setGoalCurrent(std::vector<std::string> joint_name, std::vector<float> goal_current);

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> right_arm_present_joint_angle_;
  std::vector<double> left_arm_present_joint_angle_;
  std::vector<double> present_kinematic_position_;

  /*****************************************************************************
  ** ROS Parameters
  *****************************************************************************/
  bool using_platform;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber right_joint_states_sub_;
  ros::Subscriber left_joint_states_sub_;
  ros::Subscriber right_kinematics_pose_sub_;
  ros::Subscriber left_kinematics_pose_sub_;

  std::vector<ros::Subscriber> right_gazebo_joints_subs;
  std::vector<ros::Subscriber> left_gazebo_joints_subs;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  void J1_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J2_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J3_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J4_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void wristJ1_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void wristJ2_R_commandCallback(const std_msgs::Float64::ConstPtr &msg);

  void J1_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J2_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J3_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void J4_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void wristJ1_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);
  void wristJ2_L_commandCallback(const std_msgs::Float64::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  //Added by Juan
  ros::ServiceClient set_goal_current_client;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);


  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
};

#endif //OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
