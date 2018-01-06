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

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <interactive_markers/interactive_marker_server.h>
#include <boost/bind.hpp>


#define INITIAL_X 0.0
#define INITIAL_Y 0.75
#define INITIAL_Z 0.5

moveit_msgs::CollisionObject collision_object_dynamic;
moveit_msgs::PlanningScene current_planning_scene;
moveit_msgs::ApplyPlanningScene srv;
moveit_msgs::CollisionObject collision_object_box1;
ros::Publisher test;
/*
void dynamicPoseCallBack(ros::NodeHandle &n, const m_tutorial::DynamicPose::ConstPtr &current_dynamic_pose){
  ROS_INFO("DYNAMIC OBJECT MOVING");
  ros::ServiceClient current_planning_scene_diff_client = n.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  current_planning_scene_diff_client.waitForExistence();
  
  //ros::Publisher current_planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher current_object = n.advertise<moveit_msgs::CollisionObject>("collision_object", 1000);
  //whatever comes from feedback + offset given by the initial position of the obctacle
  
  ROS_INFO("HERE");
  collision_object_dynamic.primitive_poses[0].position.x = current_dynamic_pose->X;
  collision_object_dynamic.primitive_poses[0].position.y = current_dynamic_pose->Y;
  collision_object_dynamic.primitive_poses[0].position.z = current_dynamic_pose->Z;
  collision_object_dynamic.primitive_poses[0].orientation.x = current_dynamic_pose->x;
  collision_object_dynamic.primitive_poses[0].orientation.y = current_dynamic_pose->y;
  collision_object_dynamic.primitive_poses[0].orientation.z = current_dynamic_pose->z;
  collision_object_dynamic.primitive_poses[0].orientation.w = current_dynamic_pose->w;

  collision_object_dynamic.operation = collision_object_dynamic.ADD;
  

  //current_planning_scene.is_diff = true;
  //current_planning_scene.world.collision_objects.push_back(collision_object_dynamic);
  //current_planning_scene_diff_publisher.publish(current_planning_scene);
  ROS_INFO("PUBLISHING DYNAMIC OBJECT MOVING");
  current_object.publish(collision_object_dynamic);

  test.publish(collision_object_dynamic);
  //srv.request.scene.world.collision_objects.push_back(collision_object_dynamic);
  //srv.request.scene = current_planning_scene;
  //current_planning_scene_diff_client.call(srv);
}
*/

int main(int argc, char **argv)
{
  ros::init (argc, argv, "add_obstacle_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(10.0); 
  //sleep_time.sleep();
  //sleep_time.sleep();

// Advertise the required topic
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Note that this topic may need to be remapped in the launch file --> no need to remap: the topic already exits
  ros::Publisher initial_planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  while(initial_planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  std::string frame_id = "/world"; //it has to be: "Reference frame of planning: %s", group.getPlanningFrame().c_str()); === "/world"

// Defining dynamic object: DYNAMIC
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  collision_object_dynamic.header.frame_id = "/world";
  collision_object_dynamic.id = "dynamic";

/* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive_dynamic;
  //primitive_dynamic.type = primitive_dynamic.BOX;
  primitive_dynamic.type = primitive_dynamic.SPHERE;
  primitive_dynamic.dimensions.resize(3);
  primitive_dynamic.dimensions[0] = 0.3048;
  //primitive_dynamic.dimensions[1] = 0.3048;
  //primitive_dynamic.dimensions[2] = 0.3048;

/* Define a pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose dynamic_pose;
  dynamic_pose.orientation.w = 1.0;
  dynamic_pose.position.x = 3.0;
  dynamic_pose.position.y = 3.0;
  dynamic_pose.position.z = 3.0;

/* Create collision object */
  collision_object_dynamic.primitives.push_back(primitive_dynamic);
  collision_object_dynamic.primitive_poses.push_back(dynamic_pose);
  collision_object_dynamic.operation = collision_object_dynamic.ADD;


  current_planning_scene.world.collision_objects.push_back(collision_object_dynamic);

  current_planning_scene.is_diff = true;
  current_planning_scene.robot_state.is_diff = true; // Remove if weird behaviour
  //initial_planning_scene_diff_publisher.publish(current_planning_scene);
  //sleep_time.sleep();

// * Send a diff via a rosservice call and block until
//   the diff is applied (synchronous update)
  ros::ServiceClient planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
// and send the diffs to the planning scene via a service call:
   //moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = current_planning_scene;
  if(planning_scene_diff_client.call(srv))
    ROS_INFO("diff succeed");
  else
    ROS_INFO("diff failed");

// Note that this does not continue until we are sure the diff has been applied.

  /* Sleep so we have time to see the object in RViz */
  sleep(1.0);
  
  ROS_INFO("Initial Scene SUCCESFULLY set up!");
  ROS_INFO("Automatic update of scene!");
  
  //ros::Subscriber dynamic_position = node_handle.subscribe<m_tutorial::DynamicPose>("dynamic_marker", 1000, boost::bind(&dynamicPoseCallBack, boost::ref(node_handle), _1));
  
  test = node_handle.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);
  test.publish(collision_object_dynamic);
  //Update of octomap coming from the kinect camera
  
  //ros::shutdown();
  ros::spin();
  return 0;
}
