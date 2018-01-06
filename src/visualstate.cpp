/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "spline.h"
#include <Eigen/Geometry>
#include "util_fmg.h"
#include "eigen_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualstate_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");


  
  //set up
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  
 double x,y,z,v1,v2,v3,v4,v5,v6,v2_1,v2_2,v2_3,v2_4,v2_5,v2_6;
 int disp_traj;

 node_handle.getParam("pose_x", x);
 node_handle.getParam("pose_y", y);
 node_handle.getParam("pose_z", z);

 node_handle.getParam("joint_v1", v1);
 node_handle.getParam("joint_v2", v2);
 node_handle.getParam("joint_v3", v3);
 node_handle.getParam("joint_v4", v4);
 node_handle.getParam("joint_v5", v5);
 node_handle.getParam("joint_v6", v6);

 node_handle.getParam("joint2_v1", v2_1);
 node_handle.getParam("joint2_v2", v2_2);
 node_handle.getParam("joint2_v3", v2_3);
 node_handle.getParam("joint2_v4", v2_4);
 node_handle.getParam("joint2_v5", v2_5);
 node_handle.getParam("joint2_v6", v2_6);
 node_handle.getParam("disp_traj", disp_traj);

  
  
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.




  

/// ****************visual tool
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world","/moveit_visual_markers");
  moveit_visual_tools::MoveItVisualTools visual_tools_2("world","/moveit_visual_markers");
  visual_tools.deleteAllMarkers();
  ros::Publisher traj_visualiser;
  traj_visualiser = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  //visual_tools.loadRemoteControl();
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools_2.loadRobotStatePub("/display_robot_state2");
  
  //visual_tools.trigger();




  planning_scene::PlanningScenePtr planning_scene;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
  planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
   // planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_monitor->startSceneMonitor("/planning_scene");
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene = planning_scene_monitor->getPlanningScene();

  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  moveit::planning_interface::MoveGroup mygroup("manipulator");
  //display_trajectory.trajectory_start = robot_state;
  //robot_state::robotStateToRobotStateMsg(robot_state,  display_trajectory.trajectory_start );
  std::vector<std::string> joint_names=joint_model_group->getVariableNames();
  
  Eigen::Affine3d start_state = robot_state.getGlobalLinkTransform("arm_6_link");
  ROS_INFO_STREAM("start Translation: " << start_state.translation());// Now, setup a joint space goal

  std::vector<std::vector<double> > pidpoints,interppoints;
  std::vector<double> joint_values(6, 0.0);
  pidpoints.push_back(joint_values);
  joint_values[0] = v1;
  joint_values[1] = v2;
  joint_values[2] = v3;
  joint_values[3] = v4;
  joint_values[4] = v5;
  joint_values[5] = v6; 
  pidpoints.push_back(joint_values);
  
  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  //move_group.setJointValueTarget(joint_group_positions);
  std::vector<double> joint_group_positions;
  robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);
  for(auto i : joint_group_positions)
    std::cout<<i<<" ";
  std::cout<<"****************************************************"<<std::endl;


  robot_state = planning_scene->getCurrentStateNonConst();
 visual_tools.publishRobotState(robot_state, rviz_visual_tools::GREEN);
 ros::Duration(5).sleep();


  joint_values[0] = v2_1;
  joint_values[1] = v2_2;
  joint_values[2] = v2_3;
  joint_values[3] = v2_4;
  joint_values[4] = v2_5;
  joint_values[5] = v2_6; 
  pidpoints.push_back(joint_values);

  
  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  //move_group.setJointValueTarget(joint_group_positions);
  //std::vector<double> joint_group_positions;
  robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);
  for(auto i : joint_group_positions)
    std::cout<<i<<" ";
  std::cout<<"****************************************************"<<std::endl;


  robot_state = planning_scene->getCurrentStateNonConst();
 visual_tools_2.publishRobotState(robot_state, rviz_visual_tools::BLUE);
 ros::Duration(5).sleep();
  //visual_tools.trigger();

 if(disp_traj)
 {

    /*if(!fmgplanner::cubic_interp(interppoints,pidpoints))
    {
      ROS_ERROR_STREAM("cubic first time failed!!!");
      return false;
    }
  */
  const Eigen::Affine3d &end_effector_state = robot_state.getGlobalLinkTransform("arm_6_link");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  //ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  std::vector<geometry_msgs::Pose> waypoints;

  Eigen::Affine3d pose= Eigen::Affine3d::Identity();
  
  pose.translation() = Eigen::Vector3d(0,0,0);
  geometry_msgs::Pose start_pose = fmg::toRosPose(fmg::affineToIsometry(pose));
  start_pose.position.x = start_state.translation()[0];
  start_pose.position.y = start_state.translation()[1];
  start_pose.position.z = start_state.translation()[2];


  //geometry_msgs::Pose end_effector_pose = fmg::toRosPose(fmg::affineToIsometry(pose));
  Eigen::Vector3d step = (end_effector_state.translation()-start_state.translation());
  start_pose.orientation = fmg::toRosQuaternion(Eigen::Quaterniond(end_effector_state.rotation()));
 
  for(int i =0; i < 1; i++)
  {
    waypoints.push_back(start_pose);
    start_pose.position.x += step[0];
    start_pose.position.y += step[1];
    start_pose.position.z += step[2];
    ROS_INFO_STREAM("position: " << start_pose);

  }
  waypoints.push_back(start_pose);
  //waypoints.push_back(end_effector_pose);

  /*end_effector_pose.orientation.x = -0.198;
  end_effector_pose.orientation.y = 0;
  end_effector_pose.orientation.z = 0.9;
  end_effector_pose.orientation.w = 4.2;

  
  end_effector_pose.position.x = x;
  end_effector_pose.position.y = y;
  end_effector_pose.position.z = z;

  waypoints.push_back(end_effector_pose);
  //geometry_msgs::Pose target_pose3 = start_pose2;
end_effector_pose.position.x += 0.2;
end_effector_pose.position.z += 0.2;
waypoints.push_back(end_effector_pose);  // up and out

end_effector_pose.position.y -= 0.2;
waypoints.push_back(end_effector_pose);  // left

end_effector_pose.position.z -= 0.2;
end_effector_pose.position.y += 0.2;
end_effector_pose.position.x -= 0.2;
waypoints.push_back(end_effector_pose);  // down and right (back to start)
*/
  moveit_msgs::RobotTrajectory trajectory;

  mygroup.clearPathConstraints();
  double fraction = mygroup.computeCartesianPath(waypoints,0.01,0,trajectory);
  std::cout << "farction:: " << fraction<<std::endl;


    //moveit_msgs::RobotTrajectory joint_solution = fmgplanner::toROSJointTrajectory(interppoints, joint_names, 1.0);
      ROS_INFO("Visualizing the trajectory");
      moveit_msgs::DisplayTrajectory display_trajectory;
      display_trajectory.trajectory.push_back(trajectory);
      traj_visualiser.publish(display_trajectory);
}
  

  
  ROS_INFO("Done");

  return 0;
}
