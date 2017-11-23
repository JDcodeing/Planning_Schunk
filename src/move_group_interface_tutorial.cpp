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
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "spline.h"

moveit_msgs::RobotTrajectory toROSJointTrajectory(const std::vector<std::vector<double> >& points,
                      const std::vector<std::string>& joint_names,
                      double time_delay)
{
  moveit_msgs::RobotTrajectory result;
  result.joint_trajectory.header.stamp = ros::Time::now();
  result.joint_trajectory.header.frame_id = "/world";
  result.joint_trajectory.joint_names = joint_names;

  double time_offset = 0.0;
  for(auto it = points.begin(); it!= points.end(); ++it)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = *it;
    pt.velocities.resize((*it).size(), 0.0);
    pt.accelerations.resize((*it).size(), 0.0);
    pt.effort.resize((*it).size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.joint_trajectory.points.push_back(pt);
    
  }
  return result;

}

bool cubic_interp(std::vector<std::vector<double> >& result, 
          const std::vector<std::vector<double> >& pidpoints)
{
  size_t pos_len = 0;
  if(pidpoints.size()>1)
    pos_len = pidpoints[0].size();
  else
    return false;
  if(pos_len != 6) return false;
  
  result.clear();
  for(size_t k = 0; k < pidpoints.size()*20; k++)
  {
    std::vector<double> pos(6,0.0);
    result.push_back(pos);
  }

  for(size_t i = 0; i < pos_len; i++)
  {
    std::vector<double> X,Y;
    for( size_t j = 0; j < pidpoints.size(); j++)
    {
      X.push_back(j);
      Y.push_back(pidpoints[j][i]);
    }
    tk::spline s;
    s.set_points(X,Y);
    for(int k =0; k<pidpoints.size()*20; k++)
    {
      double val  = 0.05*k;
      result[k][i] = s(val);
    }
  }
  return true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of
  // the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  std::cout << "************!!!!!planning scene done !!!***********88" << std::endl;
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.

  // We will get the name of planning plugin we want to load
  // from the ROS param server, and then load the planner
  // making sure to catch all exceptions.



  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

/// ****************visual tool
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.5; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);





  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ros::Publisher robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("display_robot_state",1);

  /* Visualize the trajectory */
  
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  //display_trajectory.trajectory_start = robot_state;
  robot_state::robotStateToRobotStateMsg(robot_state,  display_trajectory.trajectory_start );

  // Now, setup a joint space goal
  std::vector<std::vector<double> > pidpoints;
  std::vector<double> joint_values(6, 0.0);
  joint_values[0] = -0.25;
  joint_values[1] = -0.94;
  joint_values[2] = 1.3;
  joint_values[3] = 0;
  joint_values[4] = 0.7;
  joint_values[5] = 0; 
  pidpoints.push_back(joint_values);

  joint_values[0] = -0.4;
  joint_values[1] = -0.73;
  joint_values[2] = 1.7;
  joint_values[3] = -0.02;
  joint_values[4] = 0.78;
  joint_values[5] = 0; 
  pidpoints.push_back(joint_values);

  joint_values[0] = -0.26;
  joint_values[1] = -0.46;
  joint_values[2] = 2.08;
  joint_values[3] = -0.07;
  joint_values[4] = 0.92;
  joint_values[5] = 0; 
  pidpoints.push_back(joint_values);

  std::vector<std::vector<double> > result;
  
  bool cubic_suc =  cubic_interp(result,pidpoints);
  if (!cubic_suc)
  {
    std::cout<<"cubic fails!!!"<<std::endl;
    return 0;
  }
  std::vector<std::string> names;
  node_handle.getParam("joint_names", names);
  moveit_msgs::RobotTrajectory joint_solution = toROSJointTrajectory(result, names, 1.0);
  
  // Call the planner and visualize the trajectory
  /* Re-construct the planning context */

  /* Call the Planner */
  /* Check that the planning was successful */
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  display_trajectory.trajectory.push_back(joint_solution);
  std::cout << "*******************!!!!****************"<< std::endl;
  /* Now you should see two planned trajectories in series*/
  display_publisher.publish(display_trajectory);
 // std::cout << display_trajectory<<std::endl;
  sleep_time.sleep();

  ros::Rate loop_rate(1);
  size_t nodenumber = joint_solution.joint_trajectory.points.size();
  std::cout << "nodenumber:::::" << nodenumber << std::endl;
  if (nodenumber>0)
  {
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
    for (int i = 0; i < nodenumber && ros::ok(); ++i)
    {
      moveit_msgs::DisplayRobotState statemsg;
      moveit::core::jointTrajPointToRobotState(joint_solution.joint_trajectory, i, *kinematic_state);
      robot_state::robotStateToRobotStateMsg(*kinematic_state,  statemsg.state);
      robot_state_publisher.publish(statemsg);

      ros::spinOnce();
      loop_rate.sleep();


    }
      

  }
 //visual_tools.publishTrajectoryLine(joint_solution, joint_model_group);
  //visual_tools.trigger();



  /* We will add more goals. But first, set the state in the planning
     scene to the final state of the last plan */

  /* Now, we go back to the first goal*/
  // (the workspace of the robot)
  // because of this, we need to specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
  // but that is not being used in this example).
  // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
  // in this volume
  // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.

  // Call the planner and visualize all the plans created so far.

  // END_TUTORIAL
  sleep_time.sleep();
  ROS_INFO("Done");

  return 0;
}
