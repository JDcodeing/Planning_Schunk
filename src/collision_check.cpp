#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


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

bool checkStateCollision(const std::vector<double> &values, planning_scene_monitor::PlanningSceneMonitorPtr monitor){

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
    //visual_tools_->loadTrajectoryPub("/moveit_manipulation/display_trajectory");
  visual_tools_->loadMarkerPub();
    visual_tools_->setAlpha(0.8);
    visual_tools_->deleteAllMarkers();  // clear all old markers
    visual_tools_->setManualSceneUpdating(true);
    visual_tools_->loadRobotStatePub("/fgm_planner/display_robot_state");

    


  bool res = false;
    
    //moveit_msgs::PlanningScene ps_msg; 
    //ps_msg.robot_state.is_diff = true;
    //planning_scene->getPlanningSceneMsg(ps_msg);
    //planning_scene->setPlanningSceneMsg(ps_msg);
    //planning_scene_diff_publisher.publish(ps_msg);
   //ollision_detection::World &world = planning_scene_->getWorldNonConst();
    
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    
    robot_state::RobotState& current_state_r = monitor->getPlanningScene()->getCurrentStateNonConst();
    collision_request.group_name = "manipulator";
    //collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
    
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    
    std::vector<double> joint_values;

    const robot_state::JointModelGroup* joint_model_group_ = current_state_r.getJointModelGroup("manipulator");
    
    current_state_r.copyJointGroupPositions(joint_model_group_, joint_values);
    joint_values[0] = values[0];
    joint_values[1] = values[1];
    joint_values[2] = values[2];
    joint_values[3] = values[3];
    joint_values[4] = values[4];
    joint_values[5] = values[5];
    current_state_r.setVariablePositions(values);
    //current_state_r.setJointGroupPositions(joint_model_group_, joint_values);
    monitor->getPlanningScene()->setCurrentState(current_state_r); 
    visual_tools_->publishRobotState(current_state_r, rviz_visual_tools::GREEN);
    
    collision_result.clear();
    bool ha = monitor->getPlanningScene()->isStateValid(current_state_r, "manipulator");
    std::cout<< ha << "*************"<<std::endl;

    bool obj_exist = monitor->getPlanningScene()->getCollisionWorld()->getWorld()->hasObject("sphere");
    std::cout<<"check Object exists in collision world: "<<obj_exist<< std::endl;
    
    monitor->getPlanningScene()->checkCollision(collision_request, collision_result, current_state_r);
    /*ROS_INFO_STREAM("Test 4: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it){
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }*/
    current_state_r.printStatePositions(std::cout);
    if(collision_result.collision){
      res = true;
      }
    return res;
  }


int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_checking");
 	ros::AsyncSpinner spinner(1);
 	spinner.start();
 	ros::NodeHandle nh("~");

  ros::WallDuration sleep_time(5);
  
  
 	//setup
 	//robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  	//robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
 // planning_scene::PlanningScenePtr planning_scene_;

  //planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model));

  const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();

    /*planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();
    */






  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);
  //ros::ServiceClient planning_scene_diff_client =
    //  node_handle_.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
      //planning_scene_diff_client.waitForExistence();
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO("sleep");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }


  //collision_result.clear();

	moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/world";
    co.id = "sphere";
	  co.operation = moveit_msgs::CollisionObject::ADD;

	   shape_msgs::SolidPrimitive sphere;
    sphere.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere.dimensions.push_back(0.1);

    geometry_msgs::Pose pose;
  	pose.position.x = 0.28; 
  	pose.position.y = 0.44;
  	pose.position.z = 0.336;
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
      nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);

  bool obj_exist = planning_scene_monitor_->getPlanningScene()->getCollisionWorld()->getWorld()->hasObject("sphere");
  std::cout<<"Object exists in collision world: "<<obj_exist<< std::endl;

  std::vector<double> joint_values;
  joint_values.reserve(6);
  nh.getParam("jointv1",joint_values[0]);
  nh.getParam("jointv2",joint_values[1]);
  nh.getParam("jointv3",joint_values[2]);
  nh.getParam("jointv4",joint_values[3]);
  nh.getParam("jointv5",joint_values[4]);
  nh.getParam("jointv6",joint_values[5]);

  if(checkStateCollision(joint_values,planning_scene_monitor_))
    std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^collision " << std::endl;
    
    else
    {
      std::cout <<"no Collision!" <<std::endl;
    }



  	

}