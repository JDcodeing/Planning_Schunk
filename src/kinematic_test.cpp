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
std::cout <<"******************!!!!!!!!!!!!!!!!8*******************"<<std::endl;
std::cout<< x <<" "<< y  <<" "<< z << std::endl;

  //ros::WallDuration sleep_time(20.0);
  //sleep_time.sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("world","/moveit_visual_markers");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  //visual_tools.loadRemoteControl();
  visual_tools.loadRobotStatePub("/display_robot_state");


  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
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
  robot_state.copyJointGroupPositions(joint_model_group, new_joint_values);
  ROS_INFO_STREAM("new value: " << new_joint_values[0]);
  const Eigen::Affine3d &end_effector_state = robot_state.getGlobalLinkTransform("arm_6_link");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  Eigen::Affine3d pose= Eigen::Affine3d::Identity();
  
  pose.translation() = Eigen::Vector3d(x,y,z);

  geometry_msgs::Pose end_effector_pose = fmg::toRosPose(fmg::affineToIsometry(pose));
  end_effector_pose.orientation.x = -0.198;
  end_effector_pose.orientation.y = 0;
  end_effector_pose.orientation.z = 0.9;
  end_effector_pose.orientation.w = 4.2;

  //bool found_ik = robot_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1);
  
  moveit_msgs::GetPositionIK::Request ik_req;
  //moveit_msgs::RobotState moveit_rs;
  moveit_msgs::RobotState moveit_rs;
  moveit::core::robotStateToRobotStateMsg(robot_state,moveit_rs);
  std::cout<< moveit_rs << std::endl;
  ik_req.ik_request.robot_state = moveit_rs;
  ik_req.ik_request.group_name ="manipulator";

  ik_req.ik_request.avoid_collisions = true;

  geometry_msgs::PoseStamped pose_s;
  pose_s.header.stamp = ros::Time::now();
  pose_s.header.frame_id = "/world";
  pose_s.pose = end_effector_pose;

  ik_req.ik_request.pose_stamped = pose_s;
 // ik_req.ik_request.timeout = ros::Duration(0.5);
  //ik_req.ik_request.attempts = 5;


  moveit_msgs::GetPositionIK::Response ik_res;
  ROS_INFO("*******************ik calling!!!!**************");

  if(ik_service_.call(ik_req, ik_res)){
    ROS_INFO("*******************ik called!!!!**************");
    if (ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION){
      ROS_INFO("*******************no ik solution!!!!**************");
    } 
    else 
    {
       moveit::core::robotStateMsgToRobotState(ik_res.solution, robot_state);
       robot_state.copyJointGroupPositions(joint_model_group, joint_values);
       robot_state = planning_scene->getCurrentStateNonConst();
       visual_tools.publishRobotState(robot_state, rviz_visual_tools::DEFAULT);
        ros::Duration(0.1).sleep();
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("*******************found ik solution!!!!**************");
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
  }
  
  ros::shutdown();
  return 0;

}
