#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <Eigen/Geometry>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_checking");
 	ros::AsyncSpinner spinner(1);
 	spinner.start();
 	ros::NodeHandle nh;

  ros::WallDuration sleep_time(0.5);
  
  
 	//setup
 	//robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	//robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);	

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  planning_scene.checkSelfCollision(collision_request, collision_result);

  //collision_result.clear();

	moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "world";
    co.id = "sphere";
	  co.operation = moveit_msgs::CollisionObject::ADD;

	   shape_msgs::SolidPrimitive sphere;
    sphere.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere.dimensions.push_back(0.1);

    geometry_msgs::Pose pose;
  	pose.position.x = 0.6;
  	pose.position.y = 0.5;
  	pose.position.z = 1;
  	pose.orientation.w = 1.0;
  
    co.primitives.push_back(sphere);
    co.primitive_poses.push_back(pose);

  	pub_co.publish(co);
    ROS_INFO("Adding the object into the world");
    moveit_msgs::PlanningScene planning_scene;
  	planning_scene.world.collision_objects.push_back(co);
  	planning_scene.is_diff = true;
  	planning_scene_diff_publisher.publish(planning_scene);
  	sleep_time.sleep();

    ros::ServiceClient planning_scene_diff_client =
      nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
  	

}