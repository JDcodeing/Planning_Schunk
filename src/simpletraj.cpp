#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "eigen_ros.hpp"
//#include <dr_eigen/yaml.hpp>
#include <moveit_msgs/GetPositionIK.h>
#include "spline.h"
#include <string>
#include <sstream>
#include <iostream>

void printLink(const robot_state::RobotState &robot_state)
{
  ROS_INFO_STREAM("**************link*******************");
  for(int i=1; i <=6; i++)
  {
    std::ostringstream stream;

    stream<<"arm_" <<i<<"_link"; //n为int类型

    std::string link = stream.str();
    const Eigen::Affine3d &linkstate = robot_state.getGlobalLinkTransform(link);
    ROS_INFO_STREAM("the"<<i<<"th link Translation: " << linkstate.translation());
    ROS_INFO_STREAM("Rotation: " << linkstate.rotation());

  }
  
}
void printJoint(const robot_state::RobotState &robot_state)
{
  ROS_INFO_STREAM("**************joint*******************");
  for(int i=1; i <=6; i++)
  {
    std::ostringstream stream;

    stream<<"arm_" <<i<<"_joint"; //n为int类型

    std::string joint = stream.str();
    const Eigen::Affine3d &jointstate = robot_state.getJointTransform(joint);
    ROS_INFO_STREAM("the"<<i<<"th joint Translation: " << jointstate.translation());
    ROS_INFO_STREAM("Rotation: " << jointstate.rotation());

  }
  
}
void printJointPos(const robot_state::RobotState &robot_state)
{
  ROS_INFO_STREAM("**************joint pos*******************");
  for(int i=1; i <=6; i++)
  {
    std::ostringstream stream;

    stream<<"arm_" <<i<<"_joint"; //n为int类型

    std::string joint = stream.str();
    const double* jointpos = robot_state.getJointPositions(joint);
    ROS_INFO_STREAM("the"<<i<<"th joint Translation: " << jointpos);
    

  }
  
}

int main(int argc, char **argv)
{
 

  ros::init(argc, argv, "kinematics_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  //set up
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  ros::ServiceClient ik_service_= node_handle.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

  double x,y,z,v1,v2,v3,v4,v5,v6;
 node_handle.getParam("pose_x", x);
 node_handle.getParam("pose_y", y);
 node_handle.getParam("pose_z", z);

node_handle.getParam("joint_v1", v1);
 node_handle.getParam("joint_v2", v2);
 node_handle.getParam("joint_v3", v3);
 node_handle.getParam("joint_v4", v4);
 node_handle.getParam("joint_v5", v5);
 node_handle.getParam("joint_v6", v6);
 

  moveit_visual_tools::MoveItVisualTools visual_tools("world","/moveit_visual_markers");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  //visual_tools.loadRemoteControl();
  visual_tools.loadRobotStatePub("/display_robot_state");


  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();

  printLink(robot_state);
  printJoint(robot_state);
  printJointPos(robot_state);





  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  std::cout<<"^&^&^&^&^&^&^&^&^&^&^&^&joint_names " << joint_names.size()<<std::endl;
  std::vector<double> joint_values(6, 0.0);
  joint_values[0] = v1;
  joint_values[1] = v2;
  joint_values[2] = v3;
  joint_values[3] = v4;
  joint_values[4] = v5;
  joint_values[5] = v6; 

  std::vector<double> new_joint_values(6, 0.0);
  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  robot_state.update();
  robot_state.copyJointGroupPositions(joint_model_group, new_joint_values);
  //ROS_INFO_STREAM("new value: " << new_joint_values[0]);
  printLink(robot_state);
  printJoint(robot_state);
  printJointPos(robot_state);

  visual_tools.publishRobotState(robot_state, rviz_visual_tools::DEFAULT);

  //bool found_ik = robot_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1);
  
  
  ros::shutdown();
  return 0;

}
