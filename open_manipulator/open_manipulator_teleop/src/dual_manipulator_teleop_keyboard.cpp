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

/* Modified by: Juan Vallejo */

#include "open_manipulator_teleop/dual_manipulator_teleop_keyboard.h"

DualManipulatorTeleop::DualManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  right_arm_present_joint_angle_.resize(NUM_OF_JOINT);
  left_arm_present_joint_angle_.resize(NUM_OF_JOINT);
  phantom_omni_present_joint_angle.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
  priv_node_handle_.getParam("/iiwa/arm_manipulator_controller/using_platform", using_platform);
  std::cout << "Using platform teleop_keyboard: " << using_platform << std::endl;

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("DualManipulator teleoperation using keyboard start");
}

DualManipulatorTeleop::~DualManipulatorTeleop()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate DualManipulator Keyboard");
  ros::shutdown();
}

void DualManipulatorTeleop::initClient()
{
  //goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  //goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  //goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

  set_goal_current_client = node_handle_.serviceClient<open_manipulator_msgs::SetGoalCurrent>("set_goal_current");
}

void DualManipulatorTeleop::initSubscriber()
{
  if(using_platform){
    right_joint_states_sub_ = node_handle_.subscribe("/iiwa/right_arm_joint_states", 10, &DualManipulatorTeleop::jointStatesCallback, this);
    // left_joint_states_sub_ = node_handle_.subscribe("left_arm_joint_states", 10, &DualManipulatorTeleop::jointStatesCallback, this);

    phantom_omni_joint_states_sub = node_handle_.subscribe("/phantom_omni_joint_states", 10, &DualManipulatorTeleop::omniStatesCallback, this);
  }else{
    std::string right_joint_names[NUM_OF_JOINT] = {"_J1_R_", "_J2_R_", "_J3_R_", "_J4_R_", "_wristJ1_R_", "_wristJ2_R_"};
    std::string left_joint_names[NUM_OF_JOINT] = {"_J1_L_", "_J2_L_", "_J3_L_", "_J4_L_", "_wristJ1_L_", "_wristJ2_L_"};

    ros::Subscriber sub;
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[0]+"controller/command", 10, &DualManipulatorTeleop::J1_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[1]+"controller/command", 10, &DualManipulatorTeleop::J2_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[2]+"controller/command", 10, &DualManipulatorTeleop::J3_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[3]+"controller/command", 10, &DualManipulatorTeleop::J4_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[4]+"controller/command", 10, &DualManipulatorTeleop::wristJ1_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+right_joint_names[5]+"controller/command", 10, &DualManipulatorTeleop::wristJ2_R_commandCallback, this );
    right_gazebo_joints_subs.push_back(sub);

    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[0]+"controller/command", 10, &DualManipulatorTeleop::J1_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[1]+"controller/command", 10, &DualManipulatorTeleop::J2_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[2]+"controller/command", 10, &DualManipulatorTeleop::J3_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[3]+"controller/command", 10, &DualManipulatorTeleop::J4_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[4]+"controller/command", 10, &DualManipulatorTeleop::wristJ1_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
    sub = node_handle_.subscribe("EffortJointInterface"+left_joint_names[5]+"controller/command", 10, &DualManipulatorTeleop::wristJ2_L_commandCallback, this );
    left_gazebo_joints_subs.push_back(sub);
  }
  //kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &DualManipulatorTeleop::kinematicsPoseCallback, this);
}

void DualManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)// Edit for use with Dynamixels
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);

  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++){
    if (!msg->name.at(i).compare("_J1_R"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("_J2_R"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("_J3_R"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("_J4_R"))  temp_angle.at(3) = (msg->position.at(i));
  }
  right_arm_present_joint_angle_ = temp_angle;
  // Handle the left arm as well later...
}

//Called when we receive joint states from phantom omni
void DualManipulatorTeleop::omniStatesCallback(const sensor_msgs::JointState::ConstPtr &msg){
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT, 0);

  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++){
    if (!msg->name.at(i).compare("waist"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("shoulder"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("elbow"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("wrist1"))  temp_angle.at(3) = (msg->position.at(i));
  }
  phantom_omni_present_joint_angle = temp_angle;
}

void DualManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)// Edit for use with Dynamixels
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void DualManipulatorTeleop::J1_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(0) = msg->data;
  }
void DualManipulatorTeleop::J2_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(1) = msg->data; 
  }
void DualManipulatorTeleop::J3_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(2) = msg->data;
  }
void DualManipulatorTeleop::J4_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(3) = msg->data;
  }
void DualManipulatorTeleop::wristJ1_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(4) = msg->data;
  }
void DualManipulatorTeleop::wristJ2_R_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    right_arm_present_joint_angle_.at(5) = msg->data;
  }

void DualManipulatorTeleop::J1_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(0) = msg->data;
  }
void DualManipulatorTeleop::J2_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(1) = msg->data; 
  }
void DualManipulatorTeleop::J3_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(2) = msg->data;
  }
void DualManipulatorTeleop::J4_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(3) = msg->data;
  }
void DualManipulatorTeleop::wristJ1_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(4) = msg->data;
  }
void DualManipulatorTeleop::wristJ2_L_commandCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    left_arm_present_joint_angle_.at(5) = msg->data;
  }

std::vector<double> DualManipulatorTeleop::getPresentJointAngle()
{
  std::vector<double> all_joint_angles;
  all_joint_angles.insert(all_joint_angles.begin(), right_arm_present_joint_angle_.begin(), right_arm_present_joint_angle_.end());
  all_joint_angles.insert(all_joint_angles.end(), left_arm_present_joint_angle_.begin(), left_arm_present_joint_angle_.end());

  return all_joint_angles;
}

std::vector<double> DualManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool DualManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool DualManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool DualManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool DualManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool DualManipulatorTeleop::setGoalCurrent(std::vector<std::string> joint_name, std::vector<float> goal_current){
  open_manipulator_msgs::SetGoalCurrent srv;
  srv.request.goal_current = goal_current;
  srv.request.joint_name = joint_name;

  if(set_goal_current_client.call(srv)){
    return srv.response.is_planned;
  }
  return false;
}

void DualManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your DualManipulator!\n");
  printf("---------------------------\n");
  // printf("w : increase x axis in task space\n");
  // printf("s : decrease x axis in task space\n");
  // printf("a : increase y axis in task space\n");
  // printf("d : decrease y axis in task space\n");
  // printf("z : increase z axis in task space\n");
  // printf("x : decrease z axis in task space\n");
  // printf("\n");
  printf("5 : increase joint R1 angle\n");
  printf("t : decrease joint R1 angle\n");
  printf("6 : increase joint R2 angle\n");
  printf("y : decrease joint R2 angle\n");
  printf("7 : increase joint R3 angle\n");
  printf("u : decrease joint R3 angle\n");
  printf("8 : increase joint R4 angle\n");
  printf("i : decrease joint R4 angle\n");
  printf("9 : increase joint R5 angle\n");
  printf("o : decrease joint R5 angle\n");
  printf("0 : increase joint R6 angle\n");
  printf("p : decrease joint R6 angle\n");
  printf("---------------------------\n");
  printf("g : increase joint L1 angle\n");
  printf("b : decrease joint L1 angle\n");
  printf("h : increase joint L2 angle\n");
  printf("n : decrease joint L2 angle\n");
  printf("j : increase joint L3 angle\n");
  printf("m : decrease joint L3 angle\n");
  printf("k : increase joint L4 angle\n");
  printf(", : decrease joint L4 angle\n");
  printf("l : increase joint L5 angle\n");
  printf(". : decrease joint L5 angle\n");
  printf("; : increase joint L6 angle\n");
  printf("/ : decrease joint L6 angle\n");
  printf("---------------------------\n");
  std::cout << "- : increase Path Time by: " << dt << std::endl;
  std::cout << "[ : decrease Path Time by: " << dt <<std::endl;
  std::cout << " + : increase Joint Delta by " << dq << std::endl;
  std::cout << " ] : decrease Joint Delta by " << dq << std::endl;
  // printf("g : gripper open\n");
  // printf("f : gripper close\n");
  // printf("       \n");
  // printf("1 : init pose\n");
  // printf("2 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle Right Arm J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n J5: %.3lf\n J6: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3),
         getPresentJointAngle().at(4),
         getPresentJointAngle().at(5));
  printf("Present Joint Angle Left Arm J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n J5: %.3lf\n J6: %.3lf\n",
         getPresentJointAngle().at(6),
         getPresentJointAngle().at(7),
         getPresentJointAngle().at(8),
         getPresentJointAngle().at(9),
         getPresentJointAngle().at(10),
         getPresentJointAngle().at(11));
  // printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
  //        getPresentKinematicsPose().at(0),
  //        getPresentKinematicsPose().at(1),
  //        getPresentKinematicsPose().at(2));
  printf("Present Joint Delta: %.3f, Present Path Time: %.3f, Present Joint Velocity: %.3f\n",
         JOINT_DELTA, PATH_TIME, JOINT_DELTA/PATH_TIME);
  printf("---------------------------\n");

}

void DualManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT*2, 0.0);

  // if (ch == 'w' || ch == 'W')
  // {
  //   printf("input : w \tincrease(++) x axis in task space\n");
  //   goalPose.at(0) = DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // else if (ch == 's' || ch == 'S')
  // {
  //   printf("input : s \tdecrease(--) x axis in task space\n");
  //   goalPose.at(0) = -DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // else if (ch == 'a' || ch == 'A')
  // {
  //   printf("input : a \tincrease(++) y axis in task space\n");
  //   goalPose.at(1) = DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // else if (ch == 'd' || ch == 'D')
  // {
  //   printf("input : d \tdecrease(--) y axis in task space\n");
  //   goalPose.at(1) = -DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // else if (ch == 'z' || ch == 'Z')
  // {
  //   printf("input : z \tincrease(++) z axis in task space\n");
  //   goalPose.at(2) = DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // else if (ch == 'x' || ch == 'X')
  // {
  //   printf("input : x \tdecrease(--) z axis in task space\n");
  //   goalPose.at(2) = -DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  // if (ch == '5' || ch == '5')
  // {
  //   printf("input : 5 \tincrease(++) right arm joint 1 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); goalJoint.at(0) = JOINT_DELTA;
  //   joint_name.push_back("_J2_R_");
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 't' || ch == 'T')
  // {
  //   printf("input : t \tdecrease(--) right arm joint 1 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); goalJoint.at(0) = -JOINT_DELTA;
  //   joint_name.push_back("_J2_R_");
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '6' || ch == '6')
  // {
  //   printf("input : 6 \tincrease(++) right arm joint 2 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); goalJoint.at(1) = JOINT_DELTA;
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'y' || ch == 'Y')
  // {
  //   printf("input : y \tdecrease(--) right arm joint 2 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); goalJoint.at(1) = -JOINT_DELTA;
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '7' || ch == '7')
  // {
  //   printf("input : 7 \tincrease(++) right arm joint 3 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); goalJoint.at(2) = JOINT_DELTA;
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'u' || ch == 'U')
  // {
  //   printf("input : u \tdecrease(--) right arm joint 3 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); goalJoint.at(2) = -JOINT_DELTA;
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '8' || ch == '8')
  // {
  //   printf("input : 8 \tincrease(++) right arm joint 4 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); goalJoint.at(3) = JOINT_DELTA;
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'i' || ch == 'I')
  // {
  //   printf("input : i \tdecrease(--) right arm joint 4 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); goalJoint.at(3) = -JOINT_DELTA;
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '9' || ch == '9')
  // {
  //   printf("input : 9 \tincrease(++) right arm joint 5 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); goalJoint.at(4) = JOINT_DELTA;
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'o' || ch == 'O')
  // {
  //   printf("input : o \tdecrease(--) right arm joint 5 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); goalJoint.at(4) = -JOINT_DELTA;
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '0' || ch == '0')
  // {
  //   printf("input : 0 \tincrease(++) right arm joint 6 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R"); goalJoint.at(5) = JOINT_DELTA;
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'p' || ch == 'P')
  // {
  //   printf("input : p \tdecrease(--) right arm joint 6 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R"); goalJoint.at(5) = -JOINT_DELTA;
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if(ch == 'g' || ch == 'G')
  // {
  //   printf("input : g \tincrease(++) left arm joint 1 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_");
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L"); goalJoint.at(6) = JOINT_DELTA;
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'b' || ch == 'B')
  // {
  //   printf("input : b \tdecrease(--) left arm joint 1 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_");
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L"); goalJoint.at(6) = -JOINT_DELTA;
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'h' || ch == 'H')
  // {
  //   printf("input : h \tincrease(++) left arm joint 2 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_"); goalJoint.at(7) = JOINT_DELTA;
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'n' || ch == 'N')
  // {
  //   printf("input : n \tdecrease(--) left arm joint 2 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_"); goalJoint.at(7) = -JOINT_DELTA;
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'j' || ch == 'J')
  // {
  //   printf("input : j \tincrease(++) left arm joint 3 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L"); goalJoint.at(8) = JOINT_DELTA;
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'm' || ch == 'M')
  // {
  //   printf("input : m \tdecrease(--) left arm joint 3 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R");
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L"); goalJoint.at(8) = -JOINT_DELTA;
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'k' || ch == 'K')
  // {
  //   printf("input : k \tincrease(++) left arm joint 4 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L"); goalJoint.at(9) = JOINT_DELTA;
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == ',' || ch == ',')
  // {
  //   printf("input : , \tdecrease(--) left arm joint 4 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R");
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L"); goalJoint.at(9) = -JOINT_DELTA;
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == 'l' || ch == 'L')
  // {
  //   printf("input : l \tincrease(++) left arm joint 5 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L"); goalJoint.at(10) = JOINT_DELTA;
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '.' || ch == '.')
  // {
  //   printf("input : . \tdecrease(--) left arm joint 5 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R");
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L"); goalJoint.at(10) = -JOINT_DELTA;
  //   joint_name.push_back("_wristJ2_L");
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == ';' || ch == ';')
  // {
  //   printf("input : ; \tincrease(++) left arm joint 6 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R");
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R");
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R"); 
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L"); goalJoint.at(11) = JOINT_DELTA;
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  // else if (ch == '/' || ch == '/')
  // {
  //   printf("input : / \tdecrease(--) left arm joint 6 angle\n");
  //   std::vector<std::string> joint_name;
  //   joint_name.push_back("_J1_R"); 
  //   joint_name.push_back("_J2_R_"); 
  //   joint_name.push_back("_J3_R"); 
  //   joint_name.push_back("_J4_R"); 
  //   joint_name.push_back("_wristJ1_R"); 
  //   joint_name.push_back("_wristJ2_R"); 
  //   joint_name.push_back("_J1_L");
  //   joint_name.push_back("_J2_L_");
  //   joint_name.push_back("_J3_L");
  //   joint_name.push_back("_J4_L");
  //   joint_name.push_back("_wristJ1_L");
  //   joint_name.push_back("_wristJ2_L"); goalJoint.at(11) = -JOINT_DELTA;
  //   setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  // }
  if (ch == '-')
  {
    printf("input : - \tincrease(--) Path Time\n");
    PATH_TIME += dt;
  }
  else if (ch == '[')
  {
    if(PATH_TIME > 0.01){
    printf("input : [ \tdecrease(--) Path Time\n");
    PATH_TIME -= dt;
    }
    else{
      printf("Can't decrease PATH_TIME any more");
    }
  }
  else if (ch == '=')
  {
    printf("input : = \tincrease(--) Joint Delta\n");
    JOINT_DELTA += dq;
  }
  else if (ch == ']')
  {
    printf("input : ] \tincrease(--) Joint Delta\n");
    JOINT_DELTA -= dq;
  }
  // else if (ch == 'g' || ch == 'G')
  // {
  //   printf("input : g \topen gripper\n");
  //   std::vector<double> joint_angle;

  //   joint_angle.push_back(0.01);
  //   setToolControl(joint_angle);
  // }
  // else if (ch == 'f' || ch == 'F')
  // {
  //   printf("input : f \tclose gripper\n");
  //   std::vector<double> joint_angle;
  //   joint_angle.push_back(-0.01);
  //   setToolControl(joint_angle);
  // }
  // else if (ch == '2')
  // {
  //   printf("input : 2 \thome pose\n");
  //   std::vector<std::string> joint_name;
  //   std::vector<double> joint_angle;
  //   double path_time = 2.0;

  //   joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  //   joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
  //   joint_name.push_back("joint3"); joint_angle.push_back(0.35);
  //   joint_name.push_back("joint4"); joint_angle.push_back(0.70);
  //   setJointSpacePath(joint_name, joint_angle, path_time);
  // }
  // else if (ch == '1')
  // {
  //   printf("input : 1 \tinit pose\n");

  //   std::vector<std::string> joint_name;
  //   std::vector<double> joint_angle;
  //   double path_time = 2.0;
  //   joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  //   joint_name.push_back("joint2"); joint_angle.push_back(0.0);
  //   joint_name.push_back("joint3"); joint_angle.push_back(0.0);
  //   joint_name.push_back("joint4"); joint_angle.push_back(0.0);
  //   setJointSpacePath(joint_name, joint_angle, path_time);
  }

void DualManipulatorTeleop::setGoalFromOmni(){
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT*2, 0.0);
  std::vector<std::string> joint_name;

  joint_name.push_back("_J1_R"); 
  joint_name.push_back("_J2_R_"); 
  joint_name.push_back("_J3_R"); 
  joint_name.push_back("_J4_R");
  joint_name.push_back("_wristJ1_R"); 
  joint_name.push_back("_wristJ2_R"); 
  joint_name.push_back("_J1_L");
  joint_name.push_back("_J2_L_");
  joint_name.push_back("_J3_L");
  joint_name.push_back("_J4_L");
  joint_name.push_back("_wristJ1_L");
  joint_name.push_back("_wristJ2_L"); 

  ros::spinOnce();

  goalJoint.at(0) =  phantom_omni_present_joint_angle.at(0)-(right_arm_present_joint_angle_.at(0)-(-1*M_PI/2));
  goalJoint.at(1) = -(phantom_omni_present_joint_angle.at(1)-0.27)-right_arm_present_joint_angle_.at(1);
  goalJoint.at(2) = (phantom_omni_present_joint_angle.at(2) - (-0.6397))-(right_arm_present_joint_angle_.at(2)-(-2.92));
  goalJoint.at(3) = (phantom_omni_present_joint_angle.at(3)-M_PI)-(right_arm_present_joint_angle_.at(3));
  for(int i = 0; i < 4; i++){
    if(abs(goalJoint.at(i)) > 0.1){
      if(goalJoint.at(i)>0){
        goalJoint.at(i) = 0.1;
      }else{
        goalJoint.at(i) = -0.1;
      }
    }
    //std::cout << "goalJoint.at(" << i << ") = " << goalJoint.at(i) << ", ";  
  }
  //std::cout << std::endl;
  setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
}


void DualManipulatorTeleop::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void DualManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

void DualManipulatorTeleop::printPresentOmniJoints(){
  for(int i = 0; i < phantom_omni_present_joint_angle.size(); i++){
    std::cout << phantom_omni_present_joint_angle.at(i) << ", ";
  }
  std::cout << std::endl;
}

void DualManipulatorTeleop::startOmniThread(){
  int error;
  if ((error = pthread_create(&this->omni_thread, NULL, this->omniThreadRoutine, this)) != 0)
  {
    std::cout << "Creating omni thread failed!! " << (double)error << std::endl;
    exit(-1);
  }
  std::cout << "Omni Thread Created!" << std::endl;
  omni_thread_state = true;
}

void *DualManipulatorTeleop::omniThreadRoutine(void *param){
  DualManipulatorTeleop *teleop_controller = (DualManipulatorTeleop *) param;
  // static struct timespec last_time;
  // static struct timespec curr_time;

  // double delta_nsec = 0;

  // std::cout << "Entering the Thread Routine Loop" << std::endl;
  // clock_gettime(CLOCK_MONOTONIC, &last_time);
  // while(teleop_controller->omni_thread_state)
  // {

  //   std::cout << "Inside the Thread Routine Loop" << std::endl;
    
  //   while (delta_nsec<PATH_TIME){

  //     // double time = last_time.tv_sec + (next_time.tv_nsec*0.000000001);
  //     clock_gettime(CLOCK_MONOTONIC, &curr_time);
  //     /////
  //     delta_nsec = (last_time.tv_sec - curr_time.tv_sec) + ((double)(last_time.tv_nsec - curr_time.tv_nsec)*0.000000001);
  //   }
  //   /////
  // teleop_controller->setGoalFromOmni();
  // double time = last_time.tv_sec + (next_time.tv_nsec*0.000000001);
  // std::cout << "Time in thread: " << time << std::endl
  // clock_gettime(CLOCK_MONOTONIC, &last_time);
  // }
  while(teleop_controller->omni_thread_state){
    teleop_controller->setGoalFromOmni();
    usleep(10000); // Approx 100 Hz, better ays to do it (look above, couldn't figure out eactly what was wrong)
  }

  return 0;
}



int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dual_manipulator_teleop_keyboard");
  DualManipulatorTeleop dualManipulatorTeleop;

  char ch;
  dualManipulatorTeleop.printText();
  std::vector<std::string> joint_names{"_J1_R", "_J2_R","_J3_R","_J4_R"};
  std::vector<float> current{1000,1000,1000,1000};  
  dualManipulatorTeleop.setGoalCurrent(joint_names, current);
  dualManipulatorTeleop.startOmniThread();
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    dualManipulatorTeleop.printText();
    ros::spinOnce();
    dualManipulatorTeleop.setGoal(ch);
    //dualManipulatorTeleop.setGoalFromOmni();
    //Debug
    //dualManipulatorTeleop.printPresentOmniJoints();
  }

  return 0;
}
