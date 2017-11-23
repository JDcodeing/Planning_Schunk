#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <Eigen/Geometry>
#include "eigen_ros.hpp"
//#include <dr_eigen/yaml.hpp>
#include <moveit_msgs/GetPositionIK.h>
#include "spline.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
std::vector<Eigen::Vector3d> sphere_centers;
std::vector<double> sphere_radius;
std::vector<Eigen::Vector3d> Traj_pos;
std::vector<std::vector<double> > interpTraj;
std::vector<std::vector<double> > initpTraj;
const double to_obs = 0.2;
const int obs_num=3;
Eigen::Vector3d currentpos;
Eigen::Vector3d goal=Eigen::Vector3d(0.56,0.06,0.056);
Eigen::Vector3d start;
double cur2goal_dis;

class mid_info
{
  
public:
  Eigen::Vector3d pos;
  double dis;
  mid_info():pos(Eigen::Vector3d(0,0,0)),dis(0.0){};

  mid_info(Eigen::Vector3d p, double d):pos(p), dis(d){};
  
};
bool Mid_Greater( const mid_info& a, const mid_info& b)  { return a.dis > b.dis; }

bool cubic_interp(std::vector<std::vector<double> >& result, 
          const std::vector<std::vector<double> >& pidpoints)
{
  size_t pos_len = 0;
  if(pidpoints.size()>1)
    pos_len = pidpoints[0].size();
  else
    return false;
  //if(pos_len != 6) return false;
  
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
    size_t k;
    for(k =0; k<(pidpoints.size()-1)*20; k++)
    {
      double val  = 0.05*k;
      result[k][i] = s(val);
    }
    double val  = 0.05*k;
    result[k][i] = s(val);
  }
  return true;

}

bool cubic_interp_eigen(std::vector<Eigen::Vector3d>& result, 
          const std::vector<Eigen::Vector3d>& pidpoints,int samples)
{
  if(pidpoints.size()<2)
    return false;
  double stepsize = 1.0/samples;
  
  result.clear();
  for(size_t k = 0; k < (pidpoints.size()-1)*samples+1; k++)
  {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    result.push_back(pos);
  }

  for(size_t i = 0; i < 3; i++)
  {
    std::vector<double> X,Y;
    for( size_t j = 0; j < pidpoints.size(); j++)
    {
      X.push_back(j);
      Y.push_back(pidpoints[j](i));
    }
    tk::spline s;
    s.set_points(X,Y);

    for(size_t k =0; k<(pidpoints.size()-1)*samples+1; k++)
    {
      double val  = stepsize*k;
      result[k](i) = s(val);
    }
    
  }
  return true;

}

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

std::vector<mid_info> pmids_vector;
//// global 

bool isPointinObs(const Eigen::Vector3d& point)
{
  for(size_t i =0; i < obs_num; i++)
  {
      if((point - sphere_centers[i]).norm() < sphere_radius[i])
        return true;
  }
  return false;

}
void GenerateGaps()
{
  //between obs and the floor
  double dis;
  Eigen::Vector3d pos;
  for(size_t i = 0; i < obs_num; i++)
  {
    double len = sphere_centers[i](2);
    if (len <= sphere_radius[i]) continue;
    dis = (len-sphere_radius[i])*0.5;
    pos = sphere_centers[i];
    pos(2) = dis;
    if(!isPointinObs(pos))
    {
      pmids_vector.push_back(mid_info(pos,dis));
    }
  }

  // between obs
  for(size_t i = 0; i < obs_num; i++)
  {
    if(i==obs_num) 
      break;
    
    for(size_t j = i+1; j < obs_num; j++)
    {
      Eigen::Vector3d o2o = (sphere_centers[i] - sphere_centers[j]);
      dis = o2o.norm() - sphere_radius[i] - sphere_radius[j];
      pos = (dis*0.5+sphere_radius[j])*o2o.normalized()+sphere_centers[j];
      if(!isPointinObs(pos))
    {
      pmids_vector.push_back(mid_info(pos,dis));
    }

    }

  }
  std::sort(pmids_vector.begin(), pmids_vector.end(),Mid_Greater);
  for(auto i:pmids_vector)
  {
    std::cout << i.dis <<" " << i.pos(0) << " " << i.pos(1)<< " " << i.pos(2)<< std::endl; 
  }
}


void addobstacles(ros::NodeHandle& nh)
{
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO("sleep");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
  //setup
  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    moveit_msgs::CollisionObject co,co2,co3;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "world";
    co.id = "obs1";
    co.operation = moveit_msgs::CollisionObject::ADD;
    
    co2.header.stamp = ros::Time::now();
    co2.header.frame_id = "world";
    co2.id = "obs2";
    co2.operation = moveit_msgs::CollisionObject::ADD;
   
    co3.header.stamp = ros::Time::now();
    co3.header.frame_id = "world";
    co3.id = "obs3";
    co3.operation = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive sphere;
    sphere.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere.dimensions.push_back(0.1);
    sphere_radius.push_back(0.1);
    co.primitives.push_back(sphere);

    shape_msgs::SolidPrimitive sphere2;
    sphere2.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere2.dimensions.push_back(0.08);
    sphere_radius.push_back(0.08);
    co2.primitives.push_back(sphere2);

    shape_msgs::SolidPrimitive sphere3;
    sphere3.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere3.dimensions.push_back(0.08);
    sphere_radius.push_back(0.08);
    co3.primitives.push_back(sphere3);

    geometry_msgs::Pose pose;
    pose.position.x = 0.28;
    pose.position.y = -0.44;
    pose.position.z = 0.336;
    pose.orientation.w = 1.0;
    sphere_centers.push_back(Eigen::Vector3d(0.28,-0.44,0.336));
    co.primitive_poses.push_back(pose);

    pose.position.x =0.28;
    pose.position.y = 0.04;
    pose.position.z = 0.428;
    pose.orientation.w = 1.0;
    sphere_centers.push_back(Eigen::Vector3d(0.28,0.04,0.428));
    co2.primitive_poses.push_back(pose);

    pose.position.x =0.424;
    pose.position.y = -0.1;
    pose.position.z = 0.3;
    pose.orientation.w = 1.0;
    sphere_centers.push_back(Eigen::Vector3d(0.424,-0.1,0.3));
    co3.primitive_poses.push_back(pose);
  
  
    

    pub_co.publish(co);
    pub_co.publish(co2);
    pub_co.publish(co3);
    ROS_INFO("Adding the object into the world");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.world.collision_objects.push_back(co2);
    planning_scene.world.collision_objects.push_back(co3);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    //sleep_time.sleep();

    ros::ServiceClient planning_scene_diff_client =
    nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}



bool isMidFree(const Eigen::Vector3d& point)
{
  std::cout<<"&&&&&&&&&&&&&&&&&&& checking is free point:"<<point<<std::endl;

  double dis_cu2p = (point - currentpos).norm();
  if(dis_cu2p < 0.002)
    return true;
  Eigen::Vector3d dir_cu2p = (point - currentpos).normalized();
  //std::cout<<"&&&&&&&&&&&&&&&&&&& dir_cu2p:"<<dir_cu2p<<std::endl;

  for(size_t i =0; i < obs_num; i++)
  {
      //if((point - sphere_centers[i]).norm() < sphere_radius[i])
      //  return false;

      Eigen::Vector3d cur2obs = sphere_centers[i] - currentpos;
      //std::cout<<"&&&&&&&&&&&&&&&&&&& cur2obs:"<<cur2obs<<std::endl;
      double proj = cur2obs.dot(dir_cu2p);
      //std::cout<<"&&&&&&&&&&&&&&&&&&& dis_cup:"<<dis_cu2p<<" proj: "<<proj<<std::endl;
      if(proj<0) continue;
      if(proj>dis_cu2p) continue;
      Eigen::Vector3d closestpoint = proj*dir_cu2p;
      std::cout << "closest point " << closestpoint << std::endl;
      double closest_dis = cur2obs.dot(cur2obs) - proj*proj;
      //std::cout<<"&&&&&&&&&&&&&&&&&&& closest_dis:"<<closest_dis<<std::endl;
      if(closest_dis<= sphere_radius[i]*sphere_radius[i])
        return false;
  }

  std::cout << point(0)<<" " << point(1) << " " << point(2) << "is free!!";
  return true;


}

void AddMidPoint(const Eigen::Vector3d& point)
{
  std::cout<<"!!!!!!!!!!!!!!!!!!!add one mid!!" <<std::endl;

  Traj_pos.push_back(point);
  currentpos = point;
  cur2goal_dis = (currentpos - goal).norm();

}

void appendLinearWaypoints(std::vector<Eigen::Isometry3d> & output, Eigen::Isometry3d const & start, Eigen::Isometry3d const & target, int samples) {
  Eigen::Isometry3d difference = start.inverse() * target;
  Eigen::Vector3d translation = difference.translation();
  Eigen::AngleAxisd rotation{difference.rotation()};


  for (int i = 0; i < samples; ++i) {
    double factor = double(i + 1) / samples;
    Eigen::Isometry3d waypoint = start * Eigen::Translation3d{factor * translation} * Eigen::AngleAxisd{factor * rotation.angle(), rotation.axis()};
    output.push_back(waypoint);
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "myplanner init");
 	ros::AsyncSpinner spinner(1);
 	spinner.start();
 	ros::NodeHandle nh;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  ros::Publisher display_publisher =
      nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  
  moveit_msgs::DisplayTrajectory display_trajectory;

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  ros::ServiceClient ik_service_client = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  addobstacles(nh);


//**************************
 // robot_state.setJointGroupPositions(joint_model_group, joint_values);
  //move_group.setJointValueTarget(joint_group_positions);
  //std::vector<double> joint_group_positions;
  //robot_state.copyJointGroupPositions(joint_model_group, joint_group_positions);
//*************






const Eigen::Affine3d &current_state = robot_state.getGlobalLinkTransform("arm_6_link");
  ROS_INFO_STREAM("Translation: " << current_state.translation());
  currentpos = current_state.translation();
  cur2goal_dis = (currentpos-goal).norm();
  std::cout<<"!!!!!!!!!!!!!!!!!!!!current_state"<<currentpos<<std::endl;
  GenerateGaps();
  Traj_pos.push_back(currentpos);
  


  while(currentpos != goal)
  {
    if(isMidFree(goal))
    {
      std::cout << "goal is reaachable!!!" << std::endl;
      //AddMidPoint((currentpos+goal)*0.5);
      AddMidPoint(goal);

      break;
    }
    bool foundonemid = false;
    
    for(auto it = pmids_vector.begin(); it != pmids_vector.end(); ++it)
    {
      const Eigen::Vector3d p_mid= it->pos;
      if(isMidFree(p_mid))
      {
        if(cur2goal_dis > (p_mid-goal).norm())
        {
          foundonemid = true;
        //AddMidPoint((p_mid+currentpos)*0.5);
          AddMidPoint(p_mid);
          pmids_vector.erase(it);
          break;
        }
      }

    }
    if(!foundonemid)
    {
      std::cout << "Mid Point Not Found " << std::endl;
      break;
    }
  }
  std::cout << "$$$$$$$$$TRAJ $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
  for(auto ele : Traj_pos)
    std::cout<<ele(0) <<" "<<ele(1) <<" "<< ele(2) <<std::endl;

//cubic1

  

  std::vector<Eigen::Vector3d> interp_traj_pos;
  //if(cubic_interp_eigen(interp_traj_pos,Traj_pos,2))
    std::cout<<"interp_3dpos succeed!"<<std::endl;
  interp_traj_pos = Traj_pos;




  Eigen::Affine3d pose= Eigen::Affine3d::Identity();
  pose.translation() = Eigen::Vector3d::Zero();
  geometry_msgs::Pose pose_msg = dr::toRosPose(dr::affineToIsometry(pose));
  
  std::vector<double> joint_values = {0.1326,-2.2056,-1.011,-0.098,-1.203,-2.8162};


  moveit_msgs::GetPositionIK::Request ik_req;

  moveit_msgs::RobotState moveit_rs;
  

  robot_state.setJointGroupPositions(joint_model_group,joint_values);

  moveit::core::robotStateToRobotStateMsg(robot_state,moveit_rs);
  ik_req.ik_request.robot_state = moveit_rs;
  ik_req.ik_request.group_name ="manipulator";

  //ik_req.ik_request.robot_state = moveit_rs;
  ik_req.ik_request.avoid_collisions = true;
  initpTraj.clear();

  // the start point
  robot_state.copyJointGroupPositions(joint_model_group, joint_values);
  initpTraj.push_back(joint_values);
  
  ros::Rate rate(1);

  for(auto it = interp_traj_pos.begin()+1; it != interp_traj_pos.end(); it++)
  {


      pose.translation() = *it;
      std::cout <<"!!!!!!!!pose "<< pose.translation()<<std::endl;
      pose_msg = dr::toRosPose(dr::affineToIsometry(pose));
      
  
   pose_msg.orientation.x = -0.19;
   pose_msg.orientation.y = 0;
   pose_msg.orientation.z = 0.9;
   pose_msg.orientation.w = 4.2;


      geometry_msgs::PoseStamped pose_s;
      pose_s.header.stamp = ros::Time::now();
      pose_s.header.frame_id = "world";
      pose_s.pose = pose_msg;

      ik_req.ik_request.robot_state = moveit_rs;
      ik_req.ik_request.pose_stamped = pose_s;
      ik_req.ik_request.timeout = ros::Duration(1);
      ik_req.ik_request.attempts = 20;
      
      moveit_msgs::GetPositionIK::Response ik_res;

      if(ik_service_client.call(ik_req, ik_res))
      {
        if (ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION){
        ROS_INFO("*******************no ik solution!!!!**************");
        } 
      else 
        {
           moveit_rs = ik_res.solution;
           //joint_values=[0.1326,-2.2056,-1.011,-0.098,-1.203,-2.8162];
           moveit::core::robotStateMsgToRobotState(ik_res.solution, robot_state);
           robot_state.copyJointGroupPositions(joint_model_group, joint_values);
           initpTraj.push_back(joint_values);
          //for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            //ROS_INFO("***************Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
        }
      }
      rate.sleep();
  
  }

  bool cubic_suc =  cubic_interp(interpTraj,initpTraj);
  if (!cubic_suc)
  {
    std::cout<<"cubic fails!!!"<<std::endl;
    return 0;
  }

  moveit_msgs::RobotTrajectory joint_solution = toROSJointTrajectory(interpTraj, joint_names, 1.0);
  ROS_INFO("Visualizing the trajectory");
  display_trajectory.trajectory.push_back(joint_solution);
  display_publisher.publish(display_trajectory);
 // std::cout << display_trajectory<<std::endl;
  //sleep_time.sleep();
  ROS_INFO("Done");

  return 0;


  	

	 
  	

}